#include <cmath>

#include "can/can.hpp"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "tools/pid/pid.hpp"

extern "C" volatile float distance;

// 限制参数
constexpr float FOLD_THRESHOLD_CM = 30.0f;
constexpr float IMU_STATIONARY_THRESHOLD_RAD_S = 0.05f;
constexpr float FOLD_TARGET_RAD = 1.5708f;
constexpr float FOLD_MOTOR_TOLERANCE = 0.05f;
constexpr float FOLD_TASK_DT = 0.001f;

// ----------------------------------------------------
// 折叠状态
// ----------------------------------------------------
enum class FoldState
{
  WAITING,
  FOLDING,
  FOLDED,
};

FoldState fold_state = FoldState::WAITING;
// ----------------------------------------------------
// imu
// ----------------------------------------------------
bool imu_ready = false;
void imu_init()
{
  bmi088.init();
  imu_ready = true;
}
//imu数据更新与姿态解算函数
void imu_update()
{
  if (!imu_ready) {
    return;
  }
  bmi088.update();
  imu.update(bmi088.acc, bmi088.gyro);
}
// ----------------------------------------------------
// 遥控器
// ----------------------------------------------------
//遥控器初始化函数
void remote_init() { remote.request(); }

// ----------------------------------------------------
// PID
// ----------------------------------------------------
sp::PID pid_2006(FOLD_TASK_DT, 5.0f, 0.0f, 0.05f, 3.0f, 0.5f, 0.8f, false, true);
sp::PID pid_fold(FOLD_TASK_DT, 5.0f, 0.0f, 0.05f, 1.8f, 0.3f, 0.8f, false, true);
// 单环控制函数
float single_loop_control(sp::PID & pid, float target_variable, float actual_variable)
{
  pid.calc(target_variable, actual_variable);
  return pid.out;
}
// 清除折叠控制使用的 PID
void clear_control_pid()
{
  pid_2006.clear();
  pid_fold.clear();
}

// 陀螺仪存在微小噪声，因此用阈值判断三轴角速度是否接近零。
bool imu_is_stationary()
{
  return fabsf(imu.w[0]) < IMU_STATIONARY_THRESHOLD_RAD_S &&
         fabsf(imu.w[1]) < IMU_STATIONARY_THRESHOLD_RAD_S &&
         fabsf(imu.w[2]) < IMU_STATIONARY_THRESHOLD_RAD_S;
}

extern "C" void gimbal_folding_task(void const * argument)
{
  (void)argument;
  can_init();
  imu_init();
  remote_init();

  while (true) {
    float torque_2006 = 0.0f;
    float torque_fold = 0.0f;

    imu_update();
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);

    if (!motor_2006.is_open()) {
      osDelay(100);
      continue;
    }

    const sp::DBusSwitchMode left_switch = remote.sw_l;  //获取左拨杆档位

    switch (left_switch) {
      case sp::DBusSwitchMode::DOWN: {
        motor_fold.torque = 0.0f;
        motor_2006.torque = 0.0f;
        clear_control_pid();
      }

      case sp::DBusSwitchMode::MID: {
        switch (fold_state) {
          case FoldState::WAITING:
            // 只有有效测距值小于阈值时才开始收起云台。
            if (distance < FOLD_THRESHOLD_CM) {
              fold_state = FoldState::FOLDING;
              clear_control_pid();
            }
            break;

          case FoldState::FOLDING: {
            torque_fold = single_loop_control(pid_fold, FOLD_TARGET_RAD, motor_fold.angle);
            torque_2006 = single_loop_control(pid_2006, -FOLD_TARGET_RAD, motor_2006.angle);

            send_gimbal_commands(torque_2006, torque_fold, 0.0f);

            const bool fold_reached =
              fabsf(FOLD_TARGET_RAD - motor_fold.angle) < FOLD_MOTOR_TOLERANCE;
            const bool pitch_reached =
              fabsf(-FOLD_TARGET_RAD - motor_2006.angle) < FOLD_MOTOR_TOLERANCE;
            if (fold_reached && pitch_reached) {
              fold_state = FoldState::FOLDED;
            }
            break;
          }

          case FoldState::FOLDED:
            // 保持收起位置，直到 C 板 IMU 检测到机体基本静止。
            torque_fold = single_loop_control(pid_fold, FOLD_TARGET_RAD, motor_fold.angle);
            torque_2006 = single_loop_control(pid_2006, -FOLD_TARGET_RAD, motor_2006.angle);

            send_gimbal_commands(torque_2006, torque_fold, 0.0f);

            break;
        }
        case sp::DBusSwitchMode::UP: {
          torque_fold = single_loop_control(pid_fold, 0.0f, motor_fold.angle);
          torque_2006 = single_loop_control(pid_2006, 0.0f, motor_2006.angle);

          send_gimbal_commands(torque_2006, torque_fold, 0.0f);

          break;
        }
      }
    }

    osDelay(5);
  }
}
