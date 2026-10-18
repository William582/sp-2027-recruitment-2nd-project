
#include "can/can.hpp"

sp::CAN can1(&hcan1);

sp::RM_Motor motor_2006(1, sp::RM_Motors::M2006, 1.0f, true);   // M2006 #1
sp::RM_Motor motor_fold(1, sp::RM_Motors::GM6020, 1.0f, true);  // GM6020 #1
sp::RM_Motor motor_yaw(2, sp::RM_Motors::GM6020, 1.0f, true);   // GM6020 #2
sp::DBus remote(&huart3);
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);  // dt=1ms, 1kHz
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
// ============================================================
extern "C" void send_gimbal_commands(float torque_1, float torque_2, float torque_3)
{
  const float torques[3] = {torque_1, torque_2, torque_3};
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0;
  }

  motor_2006.cmd(torques[0]);
  motor_2006.write(can1.tx_data);
  can1.send(motor_2006.tx_id);  // 0x200

  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0;
  }
  motor_fold.cmd(torques[1]);
  motor_fold.write(can1.tx_data);

  motor_yaw.cmd(torques[2]);
  motor_yaw.write(can1.tx_data);
  can1.send(motor_fold.tx_id);  // 0x1FE
}
