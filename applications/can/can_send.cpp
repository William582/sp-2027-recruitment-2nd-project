#include "can/can.hpp"

namespace
{
// IMU安装旋转矩阵（C板BMI088坐标系→机器人坐标系）
// 与mecanum_wheel一致，需根据实际安装方向调整
constexpr float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
}  // namespace

// ============================================================
// 全局对象定义
// ============================================================

sp::CAN can1(&hcan1);
sp::RM_Motor motor_pitch(1, sp::RM_Motors::M2006, 1.0f, true);  // M2006 #1
sp::RM_Motor motor_fold(1, sp::RM_Motors::GM6020, 1.0f, true);  // GM6020 #1
sp::RM_Motor motor_yaw(2, sp::RM_Motors::GM6020, 1.0f, true);   // GM6020 #2

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);  // dt=1ms, 1kHz
sp::Plotter plotter(&huart1, false);

// 共享状态
extern "C" volatile int gimbal_mode = 0;
extern "C" volatile float ir_distance_cm = 0.0f;
extern "C" volatile bool ir_valid = false;

// ============================================================
// CAN初始化
// ============================================================
void can_init()
{
  can1.config();
  can1.start();
}

// ============================================================
// 发送云台电机命令
// 在taskENTER_CRITICAL内调用，防止多任务CAN冲突
// ============================================================
void can_send_gimbal()
{
  taskENTER_CRITICAL();

  // --- M2006 (0x200帧) ---
  for (int i = 0; i < 8; ++i) can1.tx_data[i] = 0;
  motor_pitch.write(can1.tx_data);
  can1.send(motor_pitch.tx_id);  // 0x200

  // --- GM6020 #1 + #2 (0x1FE帧, 共用) ---
  for (int i = 0; i < 8; ++i) can1.tx_data[i] = 0;
  motor_fold.write(can1.tx_data);  // ID=1 → data[0:1]
  motor_yaw.write(can1.tx_data);   // ID=2 → data[2:3]
  can1.send(motor_fold.tx_id);     // 0x1FE

  taskEXIT_CRITICAL();
}