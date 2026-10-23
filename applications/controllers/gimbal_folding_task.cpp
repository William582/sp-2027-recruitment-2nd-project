/**
 * ============================================================
 * 云台自动折叠任务（对应任务书 4.4.7）
 * ============================================================
 *
 * 【职责】
 *   独立 FreeRTOS 任务：控制中部 GM6020 完成回零 / 展开 / 折叠，
 *   并把力矩发布到 fold_torque，由轨迹任务统一打包进 0x1FE。
 *
 * 【为什么要有回零 HOMING】
 *   展开行程是"从折叠位算起的 130°"。如果每次上电都直接把当前角度当折叠位，
 *   那么复位时机械臂停在展开位（很常见，烧录/复位会掉力矩）就会把基准锁错，
 *   展开目标变成"再往上 130°"，电机一路顶死并持续饱和。
 *   所以使能后先朝折叠方向低力矩压到机械限位，停稳 300 ms 再以此为基准。
 *
 * 【状态机】
 *   DISABLED --(使能)------------------------------> HOMING
 *   HOMING   --(顶住限位并停稳 300ms)--------------> UNFOLDING
 *   UNFOLDING--(到位)-------------------------------> WAITING
 *   WAITING  --(触发条件满足)-----------------------> FOLDING
 *   FOLDING  --(到位)-------------------------------> FOLDED
 *   FOLDED   --(左拨杆上档)-------------------------> UNFOLDING
 *   HOMING / UNFOLDING / FOLDING --(超时或越限)-----> FAULT
 *   FAULT    --(拨杆离开上档后再重新使能)-----------> HOMING
 *
 * 【保护措施（4.4.7.3）】
 *   1. 回零用低力矩（0.45 N·m），顶到限位也不会损坏机构
 *   2. 行程限位：角度超出 [折叠位, 展开位] ±0.5 rad 立即卸载力矩
 *   3. 超时：回零 8 s、展开/折叠 3 s 未完成即进入 FAULT
 *   4. 异常退出：FAULT 输出 0 力矩；且上档模式下 FAULT 不再自动重试，
 *      必须把拨杆拨离上档才会复位，避免"一直顶着"的死循环
 *   5. 失能优先：任何一种失能判据成立时立即 DISABLED 并输出 0
 * ============================================================
 */

#include <cmath>

#include "can/can.hpp"
#include "task.h"

// 折叠轴：位置 -> 力矩的单环 PID。
// ki 抵消上段重力造成的静态下垂。
// kd 先置 0：PID 内部 dout = kd * Δangle / dt，dt=1ms 时 D 项放大约 1000 倍，
// 实测 kd=0.3 会把编码器量化噪声放大成满力矩自激抖动。抖定之后再逐步加。
sp::PID pid_fold(CONTROL_DT, 15.0f, 6.0f, 0.0f, FOLD_MAX_TORQUE, 0.6f, 0.7f, false, false);

void clear_fold_pids() { pid_fold.clear(); }

volatile FoldState fold_state = FoldState::DISABLED;
volatile float fold_torque = 0.0f;  //折叠电机的输出力矩

extern "C" void gimbal_folding_task(void const * argument)
{
  (void)argument;

  // ---- 等待折叠轴反馈，让跟踪和折叠任务同步启动 ----
  const TickType_t wait_begin = xTaskGetTickCount();
  while (!motor_fold.is_open()) {
    if (xTaskGetTickCount() - wait_begin >= pdMS_TO_TICKS(FEEDBACK_WAIT_MS)) break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  bool homed = false;          // 是否已经回零拿到基准
  uint16_t still_ticks = 0;    // 回零时"顶住不动"的连续计数
  float home_angle = 0.0f;     // 回零过程中的角度快照
  float fold_ref = 0.0f;       // 折叠位（回零得到）
  float unfold_target = 0.0f;  // 展开位
  float fold_cmd = 0.0f;       // 斜坡限速后的位置指令

  FoldState state = FoldState::DISABLED;
  TickType_t state_enter = xTaskGetTickCount();
  uint16_t confirm_ticks = 0;
  float torque = 0.0f;  //折叠电机输出力矩（局部变量）
  float fold_speed_lpf = 0.0f;
  uint16_t fold_bad_ticks = 0;  // 折叠轴反馈丢帧计数（250 ms 迟滞）

  TickType_t wake = xTaskGetTickCount();

  while (true) {
    const TickType_t now = xTaskGetTickCount();

    if (!motor_fold.is_open()) {
      // 没有任何反馈：不动作
      torque = 0.0f;
      fold_torque = torque;  //力矩置零
      state = FoldState::DISABLED;
      fold_state = state;  //状态机置为DISABLED
      state_enter = now;
      vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
      continue;
    }

    // 折叠轴反馈看门狗加 250 ms 迟滞：瞬时丢帧不应让状态机重新回零。
    // 真正的长时间掉线仍会回到 DISABLED。
    if (motor_fold.is_alive(now)) {
      fold_bad_ticks = 0;
    }
    else if (fold_bad_ticks < 300U) {
      fold_bad_ticks++;
    }
    //失能判断变量，涵盖所有失能情况，包括拨杆下档，遥控器、电机链路长时间丢帧
    const bool disabled = gimbal_is_disabled() || (fold_bad_ticks >= 250U);

    const sp::DBusSwitchMode sw_l = remote_switch_l();  //左侧拨杆档位
    const float angle = motor_fold.angle;
    fold_speed_lpf += FOLD_SPEED_LPF_ALPHA * (motor_fold.speed - fold_speed_lpf);
    const uint16_t dist = distance;  //局部变量，承接红外测距distance

    // ---------------- 状态转移 ----------------
    if (disabled) {
      if (state != FoldState::DISABLED) {
        state = FoldState::DISABLED;
        state_enter = now;
        homed = false;
        still_ticks = 0;
      }
    }
    else {
      switch (state) {
        // 从DISABLED进入HOMING
        case FoldState::DISABLED: {
          // 重新获得操作权：先回零
          state = FoldState::HOMING;
          state_enter = now;
          homed = false;
          still_ticks = 0;
          home_angle = angle;
          fold_cmd = angle;
          break;
        }

        case FoldState::HOMING: {
          // 朝折叠方向（角度减小）低力矩压过去。
          // 判据用"角度连续 300 ms 几乎不变"，比用速度阈值可靠：
          // 接近平衡时机械臂可能以极低速度慢慢爬，速度判据会误判成已到限位。
          if (fabsf(angle - home_angle) < FOLD_HOME_MOVE_EPS_RAD) {
            if (still_ticks < FOLD_HOME_STILL_TICKS) still_ticks++;
          }
          else {
            still_ticks = 0;
            home_angle = angle;
          }

          if (still_ticks >= FOLD_HOME_STILL_TICKS) {
            // 足够长的时间内电机位置没有发生移动，可以认为已经回零
            // 将此时的电机位置定为折叠位，同时定义了展开位
            fold_ref = angle;
            unfold_target = fold_ref + FOLD_UNFOLD_SIGN * FOLD_ANGLE_RAD;

            fold_cmd = fold_ref;
            // 从HOMING进入UNFOLDING
            homed = true;
            state = FoldState::UNFOLDING;

            state_enter = now;
            confirm_ticks = 0;
          }
          else if (now - state_enter > pdMS_TO_TICKS(FOLD_HOME_TIMEOUT_MS)) {
            state = FoldState::FAULT;
            state_enter = now;
          }
          break;
        }

        case FoldState::UNFOLDING: {
          if (fabsf(angle - unfold_target) <= FOLD_REACH_TOLERANCE_RAD) {
            // 从UNFOLDING进入WAITTING
            state = FoldState::WAITING;
            state_enter = now;
            confirm_ticks = 0;
          }
          else if (now - state_enter > pdMS_TO_TICKS(FOLD_TIMEOUT_MS)) {
            state = FoldState::FAULT;
            state_enter = now;
          }
          break;
        }

        case FoldState::WAITING: {
          // 只在左拨杆中档做自动折叠；上档是跟踪轨迹，不参与折叠
          bool candidate = false;
          if (sw_l != sp::DBusSwitchMode::UP) {
            // 两种折叠触发条件
            if (FOLD_TRIGGER_USE_WHEEL) {
              // 临时方案：测距模块缺失，用遥控器拨轮 ch_lu > 0.5 触发
              candidate = (remote.ch_lu > FOLD_TRIGGER_CH_LU);
            }
            else {
              // 折叠条件：1.展开后先稳定 FOLD_IR_SETTLE_MS，2.数据有效 3.测距 < 阈值。
              // 刚进入 WAITING 时机构还在动、红外可能对着地面/自身，不能立刻判断，
              // 否则会出现"刚展开就折叠"。
              const bool ir_ready =
                (now - state_enter) >= pdMS_TO_TICKS(FOLD_IR_SETTLE_MS);
              if (ir_ready) {
                candidate = (dist != IR_INVALID_CM) && (dist < FOLD_TRIGGER_CM);
              }
            }
          }
          // 连续满足need个周期后才认为状态稳定，转换为下一个状态
          const uint16_t need =
            FOLD_TRIGGER_USE_WHEEL ? FOLD_WHEEL_CONFIRM_TICKS : FOLD_CONFIRM_TICKS;

          if (candidate) {
            if (confirm_ticks < need) confirm_ticks++;
          }
          else {
            confirm_ticks = 0;
          }

          if (confirm_ticks >= need) {
            // 先进入收拢状态：等末端 M2006 收到折叠位后再真正折叠，避免干涉
            // 从UNFOLDING进入STOWING
            state = FoldState::STOWING;
            state_enter = now;
            confirm_ticks = 0;
          }
          break;
        }

        case FoldState::STOWING: {
          // 中部的 GM6020 保持展开不动，等末端 M2006 自己收到折叠位。
          // M2006 由跟踪任务控制，它看到 fold_state 不是 WAITING 就会收到 PITCH_FOLD_JOINT。
          const bool stowed = fabsf(motor_pitch.angle - PITCH_FOLD_JOINT) < PITCH_READY_TOL;
          // 从STOWING变为FOLDING
          if (stowed) {
            // M2006已进入折叠位
            state = FoldState::FOLDING;
            state_enter = now;
          }
          else if (now - state_enter > pdMS_TO_TICKS(FOLD_STOW_TIMEOUT_MS)) {
            // 收不回去也不能一直等，超时后照常折叠（避免卡死）
            state = FoldState::FOLDING;
            state_enter = now;
          }
          break;
        }

        case FoldState::FOLDING: {
          if (fabsf(angle - fold_ref) <= FOLD_REACH_TOLERANCE_RAD) {
            state = FoldState::FOLDED;
            state_enter = now;
          }
          else if (now - state_enter > pdMS_TO_TICKS(FOLD_TIMEOUT_MS)) {
            state = FoldState::FAULT;
            state_enter = now;
          }
          break;
        }

        case FoldState::FOLDED: {
          if (sw_l == sp::DBusSwitchMode::UP) {
            // 左拨杆上档：重新展开
            state = FoldState::UNFOLDING;
            state_enter = now;
            confirm_ticks = 0;
          }
          break;
        }

        case FoldState::FAULT: {
          // 上档模式下不自动重试，必须先把拨杆拨离上档，
          // 否则会形成"超时->FAULT->立刻重来"的死循环，一直顶着机构。
          if (sw_l != sp::DBusSwitchMode::UP) {
            state = FoldState::HOMING;
            state_enter = now;
            homed = false;
            still_ticks = 0;
            home_angle = angle;
          }
          break;
        }
      }
    }

    // ---------------- 行程越限保护 ----------------
    if (
      homed &&
      // 已回零、正在受控运动或保持位置”的状态
      (state == FoldState::UNFOLDING || state == FoldState::WAITING ||
       state == FoldState::STOWING || state == FoldState::FOLDING || state == FoldState::FOLDED)) {
      const float lo = (FOLD_UNFOLD_SIGN > 0.0f) ? fold_ref : unfold_target;
      const float hi = (FOLD_UNFOLD_SIGN > 0.0f) ? unfold_target : fold_ref;

      if ((angle < lo - FOLD_LIMIT_MARGIN_RAD) || (angle > hi + FOLD_LIMIT_MARGIN_RAD)) {
        state = FoldState::FAULT;
        state_enter = now;
      }
    }

    // ---------------- 输出 ----------------
    if (state == FoldState::HOMING) {
      // 朝折叠方向（角度减小）的低力矩
      torque = -FOLD_HOME_TORQUE * FOLD_UNFOLD_SIGN;
      fold_cmd = angle;
      clear_fold_pids();
    }
    else if (state == FoldState::DISABLED || state == FoldState::FAULT) {
      torque = 0.0f;
      fold_cmd = angle;
      clear_fold_pids();
    }
    else {
      const float goal =
        ((state == FoldState::FOLDING) || (state == FoldState::FOLDED)) ? fold_ref : unfold_target;

      // 位置指令斜坡限速，避免大行程时满力矩冲击机构
      const float step = FOLD_MAX_CMD_RATE * CONTROL_DT;
      if (goal > fold_cmd + step) {
        fold_cmd += step;
      }
      else if (goal < fold_cmd - step) {
        fold_cmd -= step;
      }
      else {
        fold_cmd = goal;
      }

      // 积分分离阈值取输出限幅的 60%，大行程时暂停积分累积
      pid_fold.calc(fold_cmd, angle, FOLD_MAX_TORQUE * 0.6f);

      // 用滤波后的速度做正阻尼，抑制展开过程中的小幅抖动
      // 相当于Kd的作用，但是经过滤波后的速度更平滑
      torque = pid_fold.out - FOLD_DAMPING * fold_speed_lpf;
      if (torque > FOLD_MAX_TORQUE) torque = FOLD_MAX_TORQUE;
      if (torque < -FOLD_MAX_TORQUE) torque = -FOLD_MAX_TORQUE;
    }

    fold_torque = (DEBUG_FOLD_OPEN_LOOP_TORQUE != 0.0f) ? DEBUG_FOLD_OPEN_LOOP_TORQUE : torque;
    fold_state = state;

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}
