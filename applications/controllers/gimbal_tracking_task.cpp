/**
 * ============================================================
 * 云台目标轨迹跟踪控制任务（对应任务书 4.4.8）
 * ============================================================
 *
 * 【职责划分】
 *   本任务：yaw 轴（GM6020 ID2）+ pitch 轴（M2006 ID2）+ 唯一的一处 CAN 发送
 *   折叠任务：折叠轴（GM6020 ID1），只把力矩发布到 fold_torque
 *
 * 【启动时序】
 *   1. 折叠任务把上段展开到 130°，进入 WAITING
 *   2. 本任务检测到 WAITING 后**锁存** base_latched（瞬时抖动不会清除）
 *   3. 左拨杆上档且已锁存 → 起摆跟踪，前 0.8 s 幅值从 0 线性加到 1
 *   初始化阶段**完全不动 pitch**，不再做任何标定动作。
 *
 * 【pitch 控制的零点（写死）】
 *   水平姿态对应的两个值实测后写死在 can.hpp：
 *     PITCH_ZERO_JOINT = -0.6548 rad  （M2006 关节角）
 *     IMU_ZERO_PITCH   = -0.0361 rad  （同一姿态下的 IMU 倾角，用于校验）
 *   跟踪时 target_pitch = PITCH_ZERO_JOINT + PITCH_SIGN * blend * traj.pitch，
 *   即以该水平位为 0 点做 ±30°。
 *
 * 【为什么加了看门狗迟滞】
 *   原来"链路丢失"是立即判失能，DT7 偶尔丢几帧就会让折叠任务重新回零、
 *   跟踪任务相位重置，表现为"跟踪突然停一下又自己恢复"。
 *   现在拨杆下档仍然立即失能（安全要求），只有链路抖动才走 250 ms 迟滞。
 * ============================================================
 */

#include <cmath>

#include "can/can.hpp"
#include "refer/refer.hpp"
#include "task.h"

// ============================================================
// IMU：由本任务统一更新（本任务栈更大），折叠任务只读 imu_stationary
// ============================================================
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(CONTROL_DT);

volatile bool imu_stationary = false;

// ============================================================
// PID
// ============================================================
// 位置环输出目标速度，速度环输出力矩。
// ki 先留 0；kd 也留 0（PID 内部 dout = kd*Δangle/dt，1 ms 下放大约 1000 倍，
// 编码器噪声会被放大成满力矩抖动）。
//
// yaw 是直驱（ratio=1），速度反馈噪声不被减速比缩小；pitch 经 36:1，噪声被除 36。
// 所以两轴不能共用一套参数：yaw 的速度环更软、滤波更重。
sp::PID pid_position_yaw(CONTROL_DT, 15.0f, 0.0f, 0.0f, YAW_MAX_VEL, 0.0f, 0.1f, false, false);
sp::PID pid_speed_yaw(CONTROL_DT, 0.32f, 0.1f, 0.0f, YAW_MAX_TORQUE, 0.30f, 0.1f, false, false);

sp::PID pid_position_pitch(CONTROL_DT, 14.0f, 0.0f, 0.0f, PITCH_MAX_VEL, 0.0f, 0.1f, false, false);
sp::PID pid_speed_pitch(CONTROL_DT, 0.25f, 0.3f, 0.0f, PITCH_MAX_TORQUE, 0.30f, 0.1f, false, false);

void clear_tracking_pids()
{
  pid_position_yaw.clear();
  pid_speed_yaw.clear();
  pid_position_pitch.clear();
  pid_speed_pitch.clear();
}

// ============================================================
// 绘图通道
// ============================================================
volatile float plot_yaw_target = 0.0f;
volatile float plot_yaw_actual = 0.0f;
volatile float plot_pitch_target = 0.0f;
volatile float plot_pitch_actual = 0.0f;
volatile float plot_yaw_torque = 0.0f;
volatile float plot_pitch_torque = 0.0f;
volatile float plot_fold_angle = 0.0f;
volatile float plot_fold_torque = 0.0f;
volatile float plot_motor_status = 0.0f;
volatile float plot_can_id_mask = 0.0f;
volatile float plot_rc_status = 0.0f;
volatile float plot_fold_state = 0.0f;
volatile float plot_dbus_len = 0.0f;
volatile float plot_yaw_speed = 0.0f;
volatile float plot_pitch_speed = 0.0f;
volatile float plot_imu_pitch = 0.0f;

// refer 模块输出的对准度（0~1，1 表示 1 s 内一直对准）
volatile float plot_yaw_fire = 0.0f;
volatile float plot_pitch_fire = 0.0f;

namespace
{
// yaw 用单圈编码器，航向停在 ±π 时读数会在 +π/-π 之间跳变，必须展开成连续角度
sp::AngleUnwrapper yaw_unwrap;

// 速度反馈在 1 kHz 下有明显高频抖动，直接进速度环会形成极限环，先过一阶低通。
// yaw 直驱、噪声大（实测 ±20 rad/s），alpha=0.05 约合 8 Hz；
// pitch 经 36:1，噪声小得多，用 alpha=0.12（约 19 Hz），少吃相位滞后。
float yaw_speed_lpf = 0.0f;
float pitch_speed_lpf = 0.0f;

// ---------------- 跟踪参数 ----------------
// 零点来自 can.hpp 里写死的 PITCH_ZERO_JOINT / IMU_ZERO_PITCH，
// 初始化阶段不再标定、不再移动 pitch，展开完成即 READY。
constexpr float PITCH_CMD_RANGE = 1.20f;  // pitch 关节指令相对固定零点的最大偏移
constexpr uint32_t TRACK_BLEND_MS = 800;  // 起摆平滑时间（幅值 0→1 线性）

bool imu_ready = false;
uint16_t imu_settle_ticks = 0;

void imu_update()
{
  if (!imu_ready) {
    bmi088.init();
    imu_ready = true;
    return;
  }

  bmi088.update();
  imu.update(bmi088.acc, bmi088.gyro);

  if (imu_settle_ticks < IMU_SETTLE_TICKS) {
    imu_settle_ticks++;
  }

  imu_stationary =
    (imu_settle_ticks >= IMU_SETTLE_TICKS) && (fabsf(imu.w[0]) < IMU_STATIONARY_RAD_S) &&
    (fabsf(imu.w[1]) < IMU_STATIONARY_RAD_S) && (fabsf(imu.w[2]) < IMU_STATIONARY_RAD_S);
}
}  // namespace

// ============================================================
// 任务主体
// ============================================================
extern "C" void gimbal_tracking_task(void const * argument)
{
  (void)argument;

  can_init();
  remote_init();
  clear_tracking_pids();

  // ---- 等待三台电机反馈 ----
  const TickType_t wait_begin = xTaskGetTickCount();
  while (!(motor_pitch.is_open() && motor_yaw.is_open() && motor_fold.is_open())) {
    send_gimbal_commands(0.0f, 0.0f, 0.0f);
    if (xTaskGetTickCount() - wait_begin >= pdMS_TO_TICKS(FEEDBACK_WAIT_MS)) break;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  const bool feedback_ok = motor_pitch.is_open() && motor_yaw.is_open() && motor_fold.is_open();

  float hold_yaw = 0.0f;
  float hold_pitch = 0.0f;
  float yaw_offset = 0.0f;
  uint32_t track_start_ms = 0;

  bool enabled_last = false;  //边缘检测用的锁存位(失能<->使能）
  bool tracking = false;

  // 折叠轴"已完全展开"锁存 + 反馈丢帧计数
  bool base_latched = false;
  bool track_armed = false;  // 起摆许可锁存
  uint16_t yp_bad_ticks = 0;

  // refer 模块的对准度滤波器状态：α=0.001，必须跨拍保留，不能每拍重新定义
  float yaw_fire_ratio = 0.0f;
  float pitch_fire_ratio = 0.0f;

  uint16_t id_mask_ticks = 0U;
  TickType_t wake = xTaskGetTickCount();

  while (true) {
    imu_update();

    // 展开成连续角度（消除 ±π 跳变）
    const float yaw_angle = yaw_unwrap.update(motor_yaw.angle);

    // 速度反馈一阶低通
    yaw_speed_lpf += YAW_SPEED_LPF_ALPHA * (motor_yaw.speed - yaw_speed_lpf);
    pitch_speed_lpf += PITCH_SPEED_LPF_ALPHA * (motor_pitch.speed - pitch_speed_lpf);

    const uint32_t now_ms = xTaskGetTickCount();

    // yaw/pitch 反馈看门狗：加 250 ms 迟滞，避免瞬时丢帧把力矩清零
    if (motor_yaw.is_alive(now_ms) && motor_pitch.is_alive(now_ms)) {
      yp_bad_ticks = 0;
    }
    else if (yp_bad_ticks < 300U) {
      yp_bad_ticks++;
    }
    const bool yp_alive = (yp_bad_ticks < 250U);

    const bool disabled = gimbal_is_disabled() || !feedback_ok;

    const FoldState fs = fold_state;  //读取状态副本
    const bool base_ready = (fs == FoldState::WAITING);

    // 一旦展开完成就锁存，只有真正失能才清除。
    // 这样链路抖动不会让跟踪任务重置、重新走一遍初始化。
    if (disabled) {
      base_latched = false;
      tracking = false;
    }
    else if (base_ready) {
      base_latched = true;
    }

    // ---------------- 折叠轴"完全展开"锁存 ----------------

    float torque_yaw = 0.0f;
    float torque_pitch = 0.0f;

    if (disabled) {
      // ---- 失能：最高优先级 ----
      if (enabled_last) {  //"使能→失能"的下降沿
        enabled_last = false;
        tracking = false;
        clear_tracking_pids();
      }
      hold_yaw = yaw_angle;
      hold_pitch = motor_pitch.angle;
      send_gimbal_commands(0.0f, 0.0f, 0.0f);

      plot_yaw_target = hold_yaw;
      plot_yaw_actual = yaw_angle;
      plot_pitch_target = hold_pitch;
      plot_pitch_actual = motor_pitch.angle;
      plot_yaw_torque = 0.0f;
      plot_pitch_torque = 0.0f;
    }
    else {
      if (!enabled_last) {  //"失能→使能"的上升沿
        enabled_last = true;
        tracking = false;
        hold_yaw = yaw_angle;
        hold_pitch = motor_pitch.angle;
        clear_tracking_pids();
      }

      const sp::DBusSwitchMode sw_l = remote_switch_l();
      const TrajectoryPoint traj = trajectory_read();
      // 看门狗超过250ms没有收到信号，清空pid参数
      if (!yp_alive) {
        clear_tracking_pids();
      }

      float target_yaw = hold_yaw;
      float target_pitch = hold_pitch;
      float ff_yaw_vel = 0.0f;
      float ff_yaw_acc = 0.0f;
      float ff_pitch_vel = 0.0f;
      float ff_pitch_acc = 0.0f;

      // 末端是否已找平（只在起摆许可里用一次）
      const bool pitch_ready = fabsf(motor_pitch.angle - PITCH_ZERO_JOINT) < PITCH_READY_TOL;

      // 起摆许可：左拨杆上档 + 已完全展开 + 末端已找平。
      // 这个条件只在"开始"时检查一次并锁存；一旦开始跟踪，pitch 必然会摆离
      // 零点，绝不能用它继续判断，否则会形成"摆出去→被拉回→再摆"的限制环，
      // 表现为 yaw/pitch 幅值始终很小。
      if (sw_l != sp::DBusSwitchMode::UP) {
        track_armed = false;
      }
      else if (!track_armed && base_latched && pitch_ready) {
        track_armed = true;
      }

      if (track_armed) {
        // ---- 展开完成：开始跟踪 ----
        if (!tracking) {
          tracking = true;
          track_start_ms = now_ms;
          yaw_offset = yaw_angle;  // 从当前姿态起步
          clear_tracking_pids();
        }
        // 线性软启动
        const uint32_t elapsed = now_ms - track_start_ms;
        const float blend =
          (elapsed < TRACK_BLEND_MS) ? (static_cast<float>(elapsed) / TRACK_BLEND_MS) : 1.0f;

        target_yaw = yaw_offset + blend * traj.yaw;

        // pitch：以写死的水平零点为基准做 ±30°，初始化阶段完全不动 pitch
        float pitch_cmd = PITCH_ZERO_JOINT + PITCH_SIGN * blend * traj.pitch;
        const float lo = PITCH_ZERO_JOINT - PITCH_CMD_RANGE;
        const float hi = PITCH_ZERO_JOINT + PITCH_CMD_RANGE;
        if (pitch_cmd < lo) pitch_cmd = lo;
        if (pitch_cmd > hi) pitch_cmd = hi;
        target_pitch = pitch_cmd;

        ff_yaw_vel = blend * traj.yaw_vel;
        ff_yaw_acc = blend * traj.yaw_acc;
        ff_pitch_vel = PITCH_SIGN * blend * traj.pitch_vel;
        ff_pitch_acc = PITCH_SIGN * blend * traj.pitch_acc;

        pid_position_yaw.calc(target_yaw, yaw_angle);
        const float yaw_speed_set = pid_position_yaw.out + YAW_FF_VEL * ff_yaw_vel;
        pid_speed_yaw.calc(yaw_speed_set, yaw_speed_lpf);
        torque_yaw = pid_speed_yaw.out + YAW_FF_ACC * ff_yaw_acc;

        pid_position_pitch.calc(target_pitch, motor_pitch.angle);
        const float pitch_speed_set = pid_position_pitch.out + PITCH_FF_VEL * ff_pitch_vel;
        pid_speed_pitch.calc(pitch_speed_set, pitch_speed_lpf);
        torque_pitch = pid_speed_pitch.out + PITCH_FF_ACC * ff_pitch_acc;
      }
      else {
        // ---- 中档 / 展开未完成 / 还没找平：yaw 保持，展开完成后 pitch 自动找平 ----
        if (tracking) {
          tracking = false;
          hold_yaw = yaw_angle;
          clear_tracking_pids();
        }

        target_yaw = hold_yaw;

        // 只有中部 GM6020 处于展开位时，末端 M2006 才允许摆到水平位；
        // 其余状态（折叠中/收拢中/折叠完成/展开中）都保持折叠角度，避免干涉。
        if (fs == FoldState::WAITING) {
          target_pitch = PITCH_ZERO_JOINT;  // 已展开：末端找平
        }
        else {
          target_pitch = PITCH_FOLD_JOINT;  // 折叠/收拢/展开过程：末端收在折叠位
        }

        pid_position_yaw.calc(target_yaw, yaw_angle);
        pid_speed_yaw.calc(pid_position_yaw.out, yaw_speed_lpf);
        torque_yaw = pid_speed_yaw.out;

        pid_position_pitch.calc(target_pitch, motor_pitch.angle);
        pid_speed_pitch.calc(pid_position_pitch.out, pitch_speed_lpf);
        torque_pitch = pid_speed_pitch.out;
      }

      if (!yp_alive) {
        torque_yaw = 0.0f;
        torque_pitch = 0.0f;
      }
      if (torque_yaw > YAW_MAX_TORQUE) torque_yaw = YAW_MAX_TORQUE;
      if (torque_yaw < -YAW_MAX_TORQUE) torque_yaw = -YAW_MAX_TORQUE;
      if (torque_pitch > PITCH_MAX_TORQUE) torque_pitch = PITCH_MAX_TORQUE;
      if (torque_pitch < -PITCH_MAX_TORQUE) torque_pitch = -PITCH_MAX_TORQUE;

      send_gimbal_commands(torque_pitch, fold_torque, torque_yaw);

      plot_yaw_target = target_yaw;
      plot_yaw_actual = yaw_angle;
      plot_pitch_target = target_pitch;
      plot_pitch_actual = motor_pitch.angle;
      plot_yaw_torque = torque_yaw;
      plot_pitch_torque = torque_pitch;
    }

    // 对准度检测（refer 模块）：
    //   α=0.001、调用周期 1 ms → 时间常数约 1000 ms；
    //   输出 0~1，表示最近约 1 s 内处于对准状态的比例。
    //   必须在每拍都调用（失能时目标保持当前姿态，比例会自然衰减到 0），
    //   否则暂停跟踪期间比例会冻结。
    //   实际角和规划角使用同一套坐标：yaw 用展开后的编码器角，pitch 用 M2006
    //   关节角；这样和 plot_yaw_target / plot_pitch_target 的基准一致。
    yaw_detect(yaw_angle, plot_yaw_target, &yaw_fire_ratio);
    pitch_detect(motor_pitch.angle, plot_pitch_target, &pitch_fire_ratio);
    plot_yaw_fire = yaw_fire_ratio;
    plot_pitch_fire = pitch_fire_ratio;

    // 与使能状态无关的诊断通道
    plot_fold_angle = motor_fold.angle;
    plot_fold_torque = fold_torque;
    plot_motor_status = (motor_yaw.is_alive(now_ms) ? 1.0f : 0.0f) +
                        (motor_pitch.is_alive(now_ms) ? 2.0f : 0.0f) +
                        (motor_fold.is_alive(now_ms) ? 4.0f : 0.0f);

    if (++id_mask_ticks >= 100U) {
      id_mask_ticks = 0U;
      plot_can_id_mask = static_cast<float>(g_can_id_mask);
      g_can_id_mask = 0;
    }

    if (remote.is_open()) {
      plot_rc_status = static_cast<float>(
        100 + static_cast<int>(remote.sw_r) * 10 + static_cast<int>(remote.sw_l));
    }
    else {
      plot_rc_status = 0.0f;
    }

    plot_fold_state = static_cast<float>(static_cast<int>(fold_state));
    plot_dbus_len = (g_dbus_events > 0) ? (1000.0f + static_cast<float>(g_dbus_last_len)) : 0.0f;
    plot_yaw_speed = motor_yaw.speed;
    plot_pitch_speed = motor_pitch.speed;
    plot_imu_pitch = imu.pitch;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}
