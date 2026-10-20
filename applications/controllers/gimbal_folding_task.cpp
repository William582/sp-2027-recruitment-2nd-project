#include <cmath>

#include "can/can.hpp"

// 折叠参数
constexpr float FOLD_THRESHOLD_CM = 30.0f;  //云台折叠阈值,单位厘米
constexpr float IMU_STATIONARY_THRESHOLD_RAD_S = 0.05f;
constexpr float FOLD_TARGET_ANGLE = 1.5708f;
constexpr float FOLD_MOTOR_TOLERANCE = 0.05f;  //误差阈值

// 初始化云台状态，此时应为打开
FoldState fold_state = FoldState::WAITING;

// ----------------------------------------------------
// 云台电机（M2006 ， M6020*2）
// ----------------------------------------------------
sp::RM_Motor motor_2006(1, sp::RM_Motors::M2006, 1.0f, true);    // M2006 #1
sp::RM_Motor motor_pitch(1, sp::RM_Motors::GM6020, 1.0f, true);  // GM6020 #1
sp::RM_Motor motor_yaw(2, sp::RM_Motors::GM6020, 1.0f, true);    // GM6020 #2
// ----------------------------------------------------
// imu
// ----------------------------------------------------
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);  // dt=1ms, 1kHz
bool imu_ready = false;

// imu初始化函数
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

// 陀螺仪存在微小噪声，因此用阈值判断三轴角速度是否接近零。
bool imu_is_stationary()
{
  return fabsf(imu.w[0]) < IMU_STATIONARY_THRESHOLD_RAD_S &&
         fabsf(imu.w[1]) < IMU_STATIONARY_THRESHOLD_RAD_S &&
         fabsf(imu.w[2]) < IMU_STATIONARY_THRESHOLD_RAD_S;
}
// ----------------------------------------------------
// 遥控器
// ----------------------------------------------------

//遥控器初始化函数
void remote_init() { remote.request(); }

// ----------------------------------------------------
// PID
// ----------------------------------------------------

// 单环控制函数
float single_loop_control(sp::PID & pid, float target_variable, float actual_variable)
{
  pid.calc(target_variable, actual_variable);
  return pid.out;
}

// 双环控制函数
float dual_loop_control(
  sp::PID & position_pid, sp::PID & speed_pid, float target_angle, float actual_angle,
  float actual_speed)
{
  position_pid.calc(target_angle, actual_angle);
  const float target_speed = position_pid.out;
  // 位置外环输出目标速度；速度内环输出最终控制量
  speed_pid.calc(target_speed, actual_speed);
  return speed_pid.out;
}

extern "C" void gimbal_folding_task(void const * argument)
{
  (void)argument;
  can_init();
  imu_init();
  remote_init();

  // 读取并初始化电机的角度
  HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
  const float M2006_START_ANGLE = motor_2006.angle;
  const float PITCH_START_ANGLE = motor_pitch.angle;

  while (true) {
    imu_update();
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    float torque_2006 = 0.0f;
    float torque_pitch = 0.0f;

    // 获取遥控器左拨杆档位
    const sp::DBusSwitchMode left_switch = remote.sw_l;  //获取左拨杆档位

    switch (left_switch) {
      // ----------------------------------------------------
      // DOWN：失能模式
      // ----------------------------------------------------
      case sp::DBusSwitchMode::DOWN: {
        send_gimbal_commands(0.0f, 0.0f, 0.0f);
        clear_control_pids();

        break;
      }

      // ----------------------------------------------------
      // MID：折叠准备模式（当distance小于阈值时折叠）
      // ----------------------------------------------------
      case sp::DBusSwitchMode::MID: {
        switch (fold_state) {
          // ---------
          // WAITING
          // ---------
          case FoldState::WAITING: {
            torque_2006 = dual_loop_control(
              pid_position_2006, pid_speed_2006, M2006_START_ANGLE, motor_2006.angle,
              motor_2006.speed);
            torque_pitch = dual_loop_control(
              pid_position_pitch, pid_speed_pitch, PITCH_START_ANGLE, motor_pitch.angle,
              motor_pitch.speed);
            // 当有效测距值小于阈值时更改云台状态，进入FOLD状态
            if (distance < FOLD_THRESHOLD_CM) {
              fold_state = FoldState::FOLD;
              clear_control_pids();
            }
            else {
              send_gimbal_commands(torque_2006, torque_pitch, 0.0f);
              continue;
            }
            break;
          }
          // ---------
          // FOLD
          // ---------
          case FoldState::FOLD: {
            torque_2006 = dual_loop_control(
              pid_position_2006, pid_speed_2006, M2006_START_ANGLE, motor_2006.angle,
              motor_2006.speed);
            torque_pitch = dual_loop_control(
              pid_position_pitch, pid_speed_pitch, PITCH_START_ANGLE + FOLD_TARGET_ANGLE,
              motor_pitch.angle, motor_pitch.speed);

            send_gimbal_commands(torque_2006, torque_pitch, 0.0f);

            break;
          }
          default:
            break;
        }  // switch (fold_state)
        break;
      }
      case sp::DBusSwitchMode::UP: {
        torque_2006 = dual_loop_control(
          pid_position_2006, pid_speed_2006, M2006_START_ANGLE, motor_2006.angle, motor_2006.speed);
        torque_pitch = dual_loop_control(
          pid_position_pitch, pid_speed_pitch, PITCH_START_ANGLE, motor_pitch.angle,
          motor_pitch.speed);

        send_gimbal_commands(torque_2006, torque_pitch, 0.0f);

        if (
          fabsf(motor_2006.angle - M2006_START_ANGLE) < FOLD_MOTOR_TOLERANCE &&
          fabsf(motor_pitch.angle - PITCH_START_ANGLE) < FOLD_MOTOR_TOLERANCE)
          fold_state = FoldState::UNFOLD;

        break;
      }
    }  //switch (left_switch)
    osDelay(10);
  }  //while(true)

}  //任务函数
