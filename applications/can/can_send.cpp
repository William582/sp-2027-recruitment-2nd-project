
#include "can/can.hpp"

sp::CAN can1(&hcan1);
sp::DBus remote(&huart3);

// ============================================================
// CAN通信初始化
// ============================================================
void can_init()
{
  can1.config();
  can1.start();
}

// ============================================================
// 发送云台电机命令
// ============================================================
constexpr size_t MOTOR_COUNT = 3;

void send_gimbal_commands(float torque_1, float torque_2, float torque_3)
{
  const float torques[MOTOR_COUNT] = {torque_1, torque_2, torque_3};
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0;
  }
  // 先发2006电机的指令
  motor_2006.cmd(torques[0]);
  motor_2006.write(can1.tx_data);

  can1.send(motor_2006.tx_id);  // 0x200
  // 清空发送缓冲区
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0;
  }
  // 再发两个6020电机的指令
  motor_pitch.cmd(torques[1]);
  motor_pitch.write(can1.tx_data);

  motor_yaw.cmd(torques[2]);
  motor_yaw.write(can1.tx_data);

  can1.send(motor_pitch.tx_id);  // 0x1FE
}
