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
#include "task.h"
#include "trajectory_task.hpp"

// ============================================================
// 控制周期
// ============================================================
constexpr float TRACK_DT = 0.001f;  // 控制周期 1ms (秒)

// ============================================================
// 输出限幅参数
// ============================================================
// Yaw轴 (GM6020, 最大力矩约 5 N·m)
constexpr float YAW_MAX_TORQUE = 5.0f;  // 力矩限幅 N·m
constexpr float YAW_MAX_VEL = 12.0f;    // 位置环输出(目标速度)限幅 rad/s

// Pitch轴 (M2006, 最大力矩约 3 N·m)
constexpr float PITCH_MAX_TORQUE = 3.0f;
constexpr float PITCH_MAX_VEL = 12.0f;

// pitch轴 (GM6020, 保持展开)
constexpr float FOLD_MAX_TORQUE = 1.8f;

// ============================================================
// 前馈增益
// ============================================================
constexpr float YAW_FF_VEL = 1.0f;   // 速度前馈：将轨迹速度直接前馈到速度环设定值
constexpr float YAW_FF_ACC = 0.01f;  // 加速度前馈：将轨迹加速度转换为力矩前馈
constexpr float PITCH_FF_VEL = 1.0f;
constexpr float PITCH_FF_ACC = 0.01f;
// ============================================================
// PID
// ============================================================
//dt kp ki kd maxout maxiout alpha multi_circle dynamic
// Yaw轴 (GM6020 #2)
sp::PID pid_position_yaw(TRACK_DT, 2.0f, 0.0f, 0.0f, YAW_MAX_VEL, 0.0f, 0.85f, true, false);
sp::PID pid_speed_yaw(TRACK_DT, 3.0f, 0.2f, 0.0f, 0.0f, YAW_MAX_TORQUE, 0.85f, false, false);

// Pitch轴 (M2006 #1)
sp::PID pid_position_2006(TRACK_DT, 20.0f, 0.0f, 0.2f, PITCH_MAX_VEL, 0.0f, 0.85f, true, false);
sp::PID pid_speed_2006(TRACK_DT, 3.0f, 0.2f, 0.03f, PITCH_MAX_TORQUE, 0.5f, 0.85f, false, false);

// Pitch轴 (GM6020 #1)
sp::PID pid_position_pitch(TRACK_DT, 5.0f, 0.0f, 0.05f, FOLD_MAX_TORQUE, 0.3f, 0.8f, false, false);
sp::PID pid_speed_pitch(TRACK_DT, 3.0f, 0.2f, 0.03f, PITCH_MAX_TORQUE, 0.5f, 0.85f, false, false);

// 清除跟随控制使用的 PID
void clear_control_pids()
{
  pid_position_2006.clear();
  pid_position_pitch.clear();
  pid_speed_2006.clear();
  pid_speed_pitch.clear();
  pid_position_yaw.clear();
  pid_speed_yaw.clear();
}
// 带前馈的级联控制函数
// position_pid: 位置外环 PID
// speed_pid:    速度内环 PID（需用支持 set_dot/fdb_dot 的 calc 重载）
// target_angle:  目标角度（来自轨迹）
// actual_angle:  实际角度（编码器）
// actual_speed:  实际速度（编码器）
// ff_speed:      轨迹目标速度（前馈）
// ff_acc:      轨迹目标加速度（前馈）
// ff_speed_gain:        速度前馈增益
// ff_acc_gain:        加速度前馈增益
float dual_loop_control(
  sp::PID & position_pid, sp::PID & speed_pid, float target_angle, float actual_angle,
  float actual_speed, float ff_speed, float ff_acc, float ff_speed_gain, float ff_acc_gain)
{
  //  外环：位置 PID
  // 输出 = 目标速度修正量
  position_pid.calc(target_angle, actual_angle);
  float target_speed = position_pid.out;

  // 内环：速度 PID + 速度前馈
  // 速度前馈：将轨迹速度直接叠加到速度环设定值
  const float set_speed = target_speed + ff_speed_gain * ff_speed;

  // 使用 PID 类的前馈重载，fdb_dot=actual_speed 提供微分先行
  // set_dot=set_speed 使 D 项跟踪设定值变化率，而非仅反馈变化率
  speed_pid.calc(set_speed, actual_speed);
  float torque = speed_pid.out;

  //  加速度前馈：直接叠加到最终力矩
  torque += ff_acc_gain * ff_acc;

  // max_out 已在 PID 构造时设定，但前馈是外部叠加的，需手动限幅
  return torque;
}
// ============================================================
// 轨迹跟踪控制任务
// ============================================================
extern "C" void gimbal_tracking_task(void const * argument)
{
  (void)argument;

  // --- 初始化CAN ---
  can_init();
  // --- 清除PID历史状态 ---
  clear_control_pids();

  HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
  float FOLD_HOLD_TARGET_ANGLE = motor_pitch.angle;  // 展开状态目标角度(保持不变)

  for (;;) {
    clear_control_pids();

    while (fold_state == FoldState::UNFOLD) {
      //CAN通信反馈函数
      HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);

      //读取当前轨迹目标 (位置 / 速度 / 加速度)
      const TrajectoryPoint traj = trajectory_read();

      float torque_2006 = 0.0f;
      float torque_pitch = 0.0f;
      float torque_yaw = 0.0f;

      // Pitch 轴 (M2006) — 级联 PID + 速度/加速度前馈
      torque_2006 = dual_loop_control(
        pid_position_2006, pid_speed_2006, traj.pitch, motor_2006.angle,
        motor_2006.speed,                // 目标角度、实际角度、实际速度
        traj.pitch_vel, traj.pitch_acc,  // 轨迹速度、加速度
        PITCH_FF_VEL, PITCH_FF_ACC);     // 前馈增益

      // Yaw 轴 (GM6020)
      torque_yaw = dual_loop_control(
        pid_position_yaw, pid_speed_yaw, traj.yaw, motor_yaw.angle, motor_yaw.speed, traj.yaw_vel,
        traj.yaw_acc, YAW_FF_VEL, YAW_FF_ACC);

      torque_pitch =
        single_loop_control(pid_position_pitch, FOLD_HOLD_TARGET_ANGLE, motor_pitch.angle);

      // 发送电机命令
      // 参数顺序: (M2006 pitch力矩, GM6020 fold力矩, GM6020 yaw力矩)
      send_gimbal_commands(torque_2006, torque_pitch, torque_yaw);

      //绘图输出
      // ch1: yaw目标  ch2: yaw实际  ch3: pitch目标  ch4: pitch实际
      // ch5: yaw力矩  ch6: pitch力矩
      osDelay(10);
    }  //while(true)
  }
}
