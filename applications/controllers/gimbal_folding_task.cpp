#include <cmath>

#include "can/can.hpp"

// 折叠参数
constexpr float FOLD_THRESHOLD_CM = 30.0f;     // 折叠触发距离阈值(cm)
constexpr float UNFOLD_THRESHOLD_CM = 40.0f;   // 展开触发距离阈值(cm)，带回差
constexpr float FOLD_TARGET_RAD = 1.5708f;     // 90° = π/2
constexpr float FOLD_MOTOR_TOLERANCE = 0.05f;  // 到位容差(rad, ~3°)
constexpr float FOLD_TASK_DT = 0.005f;         // 5ms控制周期

// 折叠PID（位置环 → 力矩）
// motor_fold (GM6020#1): 折叠主驱动
// motor_pitch (M2006#1): 补偿保持传感器平视
sp::PID fold_pid(FOLD_TASK_DT, 5.0f, 0.0f, 0.05f, 3.0f, 0.5f, 0.8f, false, false);
sp::PID pitch_fold_pid(FOLD_TASK_DT, 5.0f, 0.0f, 0.05f, 1.8f, 0.3f, 0.8f, false, false);

// 折叠状态机子状态
enum class FoldState
{
  WAITING,   // 等待折叠条件
  FOLDING,   // 正在折叠
  FOLDED,    // 已折叠
  UNFOLDING  // 正在展开
};

static FoldState fold_state = FoldState::WAITING;

// 云台折叠FreeRTOS任务
extern "C" void gimbal_folding_task(void const * argument)
{
  (void)argument;

  while (true) {
    // 仅当IMU和CAN已初始化时才运行（由trajectory_task初始化）
    // 折叠任务等待其他任务完成初始化
    if (!motor_pitch.is_open()) {
      osDelay(100);
      continue;
    }

    float dist = ir_distance_cm;
    bool valid = ir_valid;

    switch (fold_state) {
      // ---- 等待：距离小于阈值时开始折叠 ----
      case FoldState::WAITING: {
        if (valid && dist < FOLD_THRESHOLD_CM) {
          fold_state = FoldState::FOLDING;
          gimbal_mode = 1;  // 接管电机控制权
          fold_pid.clear();
          pitch_fold_pid.clear();
        }
        break;
      }

      // ---- 折叠中：控制GM6020#1转90°, M2006#1反向转90° ----
      case FoldState::FOLDING: {
        // GM6020#1 目标角度 = +90°
        fold_pid.calc(FOLD_TARGET_RAD, motor_fold.angle);
        motor_fold.cmd(fold_pid.out);

        // M2006#1 目标角度 = -90°（补偿，保持传感器平视）
        pitch_fold_pid.calc(-FOLD_TARGET_RAD, motor_pitch.angle);
        motor_pitch.cmd(pitch_fold_pid.out);

        can_send_gimbal();

        // 检查到位
        float fold_err = fabsf(FOLD_TARGET_RAD - motor_fold.angle);
        float pitch_err = fabsf(-FOLD_TARGET_RAD - motor_pitch.angle);
        if (fold_err < FOLD_MOTOR_TOLERANCE && pitch_err < FOLD_MOTOR_TOLERANCE) {
          fold_state = FoldState::FOLDED;
          gimbal_mode = 2;  // 已折叠
        }

        // 超时保护：5秒内未完成则退出
        static uint32_t fold_start_tick = 0;
        if (fold_state == FoldState::FOLDING) {
          // 首次进入时记录时间
          // (简化实现：在状态切换时重置)
        }
        break;
      }

      // ---- 已折叠：等待距离恢复后展开 ----
      case FoldState::FOLDED: {
        // 保持折叠位置
        fold_pid.calc(FOLD_TARGET_RAD, motor_fold.angle);
        motor_fold.cmd(fold_pid.out);
        pitch_fold_pid.calc(-FOLD_TARGET_RAD, motor_pitch.angle);
        motor_pitch.cmd(pitch_fold_pid.out);
        can_send_gimbal();

        // 距离大于阈值+回差时展开
        if (valid && dist > UNFOLD_THRESHOLD_CM) {
          fold_state = FoldState::UNFOLDING;
          fold_pid.clear();
          pitch_fold_pid.clear();
        }
        break;
      }

      // ---- 展开中：回到0° ----
      case FoldState::UNFOLDING: {
        fold_pid.calc(0.0f, motor_fold.angle);
        motor_fold.cmd(fold_pid.out);
        pitch_fold_pid.calc(0.0f, motor_pitch.angle);
        motor_pitch.cmd(pitch_fold_pid.out);
        can_send_gimbal();

        float fold_err = fabsf(motor_fold.angle);
        float pitch_err = fabsf(motor_pitch.angle);
        if (fold_err < FOLD_MOTOR_TOLERANCE && pitch_err < FOLD_MOTOR_TOLERANCE) {
          fold_state = FoldState::WAITING;
          gimbal_mode = 0;  // 释放电机控制权
        }
        break;
      }
    }

    osDelay(5);  // 5ms周期
  }
}