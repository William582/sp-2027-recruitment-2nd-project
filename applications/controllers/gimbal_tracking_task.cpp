/**
 * ============================================================
 * 云台轨迹跟踪控制任务
 * ============================================================
 *
 * 【任务概述】
 *   新建 FreeRTOS 任务，加载 trajectory_task 提供的目标轨迹，
 *   控制云台 yaw/pitch 两轴跟踪目标值，fold 轴保持展开。
 *
 * 【控制器结构 — 级联PID + 前馈控制】
 *
 *   Yaw轴 (GM6020 #2, motor_yaw):
 *     外环(位置): PID(target_yaw, motor_yaw.angle)          → 目标速度修正
 *     内环(速度): PID(目标速度 + 轨迹速度前馈, motor_yaw.speed) → 力矩
 *     前馈:       加速度前馈 Kff_acc * traj.yaw_acc 叠加到力矩
 *
 *   Pitch轴 (M2006 #1, motor_2006):
 *     外环(位置): PID(target_pitch, motor_2006.angle)        → 目标速度修正
 *     内环(速度): PID(目标速度 + 轨迹速度前馈, motor_2006.speed) → 力矩
 *     前馈:       加速度前馈 Kff_acc * traj.pitch_acc 叠加到力矩
 *
 *   Fold轴 (GM6020 #1, motor_fold):
 *     单环位置PID，目标角度=0（展开状态）
 *
 *   反馈源：电机编码器多圈角度 + 编码器速度
 *
 * 【参数整定过程】
 *   1. 位置环 ki=0，从小 kp 开始增大，直到位置响应足够快但不振荡
 *   2. 速度环从适中 kp 开始，加入 ki 消除稳态速度误差，kd 抑制超调
 *   3. 逐步增大前馈系数 YAW_FF_VEL / YAW_FF_ACC，观察跟踪误差下降
 *   4. D项低通滤波系数 alpha 根据编码器噪声情况调整(0.8~0.95)
 *   5. 最终通过上位机绘图观察目标-实际曲线重合度，微调至满意
 *
 * 【输出限幅】
 *   - 位置环输出限幅: ±YAW_MAX_VEL / ±PITCH_MAX_VEL (rad/s)
 *   - 速度环输出限幅: ±YAW_MAX_TORQUE / ±PITCH_MAX_TORQUE (N·m)
 *   - 前馈力矩叠加后再次限幅，确保不超出电机物理能力
 *
 *
 * 【绘图输出 — 6通道】
 *   ch1: yaw目标角度    ch2: yaw实际角度
 *   ch3: pitch目标角度  ch4: pitch实际角度
 *   ch5: yaw控制力矩    ch6: pitch控制力矩
 * ============================================================
 */

#include "can/can.hpp"
#include "cmsis_os.h"
#include "task.h"
#include "trajectory_task.hpp"

// ============================================================
// 控制周期
// ============================================================
constexpr float TRACK_DT = 0.001f;        // 控制周期 1ms (秒)
constexpr uint32_t TRACK_PERIOD_MS = 1U;  // 任务周期 1ms

// ============================================================
// 安装方向系数
// 如轨迹正方向与电机正方向相反，改为 -1.0f
// ============================================================
constexpr float YAW_SIGN = 1.0f;
constexpr float PITCH_SIGN = 1.0f;

// ============================================================
// 输出限幅参数
// ============================================================
// Yaw轴 (GM6020, 最大力矩约 5 N·m)
constexpr float YAW_MAX_TORQUE = 5.0f;  // 力矩限幅 N·m
constexpr float YAW_MAX_VEL = 12.0f;    // 位置环输出(目标速度)限幅 rad/s

// Pitch轴 (M2006, 最大力矩约 3 N·m)
constexpr float PITCH_MAX_TORQUE = 3.0f;
constexpr float PITCH_MAX_VEL = 12.0f;

// Fold轴 (GM6020, 保持展开)
constexpr float FOLD_MAX_TORQUE = 1.8f;
constexpr float FOLD_HOLD_TARGET_RAD = 0.0f;  // 展开状态目标角度

// ============================================================
// 前馈增益
// ============================================================
constexpr float YAW_FF_VEL = 1.0f;   // 速度前馈：将轨迹速度直接前馈到速度环设定值
constexpr float YAW_FF_ACC = 0.01f;  // 加速度前馈：将轨迹加速度转换为力矩前馈
constexpr float PITCH_FF_VEL = 1.0f;
constexpr float PITCH_FF_ACC = 0.01f;

// ============================================================
// 级联PID — Yaw轴 (GM6020 #2)
// ============================================================
// 位置环：输入角度(rad)，输出目标速度(rad/s)
sp::PID pid_yaw_pos(
  TRACK_DT,
  20.0f,        // kp: 位置响应增益
  0.0f,         // ki: 位置环不使用积分(由速度环承担)
  0.0f,         // kd: 抑制位置振荡
  YAW_MAX_VEL,  // max_out: 输出速度限幅
  0.0f,         // max_iout
  0.85f,        // alpha: D项低通滤波系数 (0~1, 越小滤波越强)
  true,         // angular: 角度环绕处理(±π跳变)
  false);       // dynamic: 不使用变速积分

// 速度环：输入速度(rad/s)，输出力矩(N·m)
sp::PID pid_yaw_vel(
  TRACK_DT,
  3.0f,            // kp: 速度响应增益
  0.2f,            // ki: 消除稳态速度误差
  0.03f,           // kd: 速度阻尼
  YAW_MAX_TORQUE,  // max_out: 输出力矩限幅
  0.5f,            // max_iout: 积分项限幅
  0.85f,           // alpha
  false,           // angular: 速度不做环绕处理
  false);          // dynamic

// ============================================================
// 级联PID — Pitch轴 (M2006 #1)
// ============================================================
sp::PID pid_pitch_pos(TRACK_DT, 20.0f, 0.0f, 0.2f, PITCH_MAX_VEL, 0.0f, 0.85f, true, false);

sp::PID pid_pitch_vel(TRACK_DT, 3.0f, 0.2f, 0.03f, PITCH_MAX_TORQUE, 0.5f, 0.85f, false, false);

// ============================================================
// 单环PID — Fold轴 (GM6020 #1, 保持展开)
// ============================================================
// sp::PID pid_fold(TRACK_DT, 5.0f, 0.0f, 0.05f, FOLD_MAX_TORQUE, 0.3f, 0.8f, false, true);

// ============================================================
// 辅助函数：对称限幅
// ============================================================
static inline float clampf(float x, float limit)
{
  if (x > limit) return limit;
  if (x < -limit) return -limit;
  return x;
}

// ============================================================
// 轨迹跟踪控制任务
// ============================================================
extern "C" void gimbal_tracking_task(void const * argument)
{
  (void)argument;

  // --- 初始化CAN ---
  can_init();

  // --- 等待 yaw/pitch 电机反馈上线 ---
  while (!motor_yaw.is_open() || !motor_2006.is_open()) {
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    osDelay(10);
  }

  // --- 清除PID历史状态 ---
  pid_yaw_pos.clear();
  pid_yaw_vel.clear();
  pid_pitch_pos.clear();
  pid_pitch_vel.clear();
  // pid_fold.clear();

  // --- 计算位置偏移 ---
  // 读取轨迹起始点，使任务启动瞬间目标=当前角度，避免初始阶跃冲击
  const TrajectoryPoint traj0 = trajectory_read();
  const float yaw_offset = motor_yaw.angle - YAW_SIGN * traj0.yaw;
  const float pitch_offset = motor_2006.angle - PITCH_SIGN * traj0.pitch;

  // --- 精确周期控制循环 (1ms) ---
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    // 1. 触发CAN接收，刷新电机编码器反馈
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);

    // 2. 读取当前轨迹目标 (位置 / 速度 / 加速度)
    const TrajectoryPoint traj = trajectory_read();

    // ===== 3. Yaw轴 级联PID + 前馈 =====
    // 3a. 外环(位置)：PID(目标角度, 编码器角度) → 目标速度修正量
    const float yaw_target = YAW_SIGN * traj.yaw + yaw_offset;
    pid_yaw_pos.calc(yaw_target, motor_yaw.angle);
    // 3b. 叠加轨迹速度前馈 → 完整目标速度
    const float yaw_vel_set = pid_yaw_pos.out + YAW_FF_VEL * YAW_SIGN * traj.yaw_vel;
    // 3c. 内环(速度)：PID(目标速度, 编码器速度) → 力矩
    pid_yaw_vel.calc(yaw_vel_set, motor_yaw.speed);
    // 3d. 叠加加速度前馈并限幅
    const float yaw_torque =
      clampf(pid_yaw_vel.out + YAW_FF_ACC * YAW_SIGN * traj.yaw_acc, YAW_MAX_TORQUE);

    // ===== 4. Pitch轴 级联PID + 前馈 =====
    const float pitch_target = PITCH_SIGN * traj.pitch + pitch_offset;
    pid_pitch_pos.calc(pitch_target, motor_2006.angle);
    const float pitch_vel_set = pid_pitch_pos.out + PITCH_FF_VEL * PITCH_SIGN * traj.pitch_vel;
    pid_pitch_vel.calc(pitch_vel_set, motor_2006.speed);
    const float pitch_torque =
      clampf(pid_pitch_vel.out + PITCH_FF_ACC * PITCH_SIGN * traj.pitch_acc, PITCH_MAX_TORQUE);

    // ===== 5. Fold轴 保持展开状态 =====
    // pid_fold.calc(FOLD_HOLD_TARGET_RAD, motor_fold.angle);
    // const float fold_torque = pid_fold.out;

    // ===== 6. 发送电机命令 =====
    // 参数顺序: (M2006 pitch力矩, GM6020 fold力矩, GM6020 yaw力矩)
    send_gimbal_commands(pitch_torque, 0.0f, yaw_torque);

    // ===== 7. 绘图输出 =====
    // ch1: yaw目标  ch2: yaw实际  ch3: pitch目标  ch4: pitch实际
    // ch5: yaw力矩  ch6: pitch力矩
    plotter.plot(
      yaw_target, motor_yaw.angle, pitch_target, motor_2006.angle, yaw_torque, pitch_torque);

    // ===== 8. 1ms精确周期 =====
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TRACK_PERIOD_MS));
  }
}
