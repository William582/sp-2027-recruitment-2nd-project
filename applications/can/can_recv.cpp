#include "can/can.hpp"

// CAN通信反馈函数
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      // 分发到对应电机
      if (can1.rx_id == motor_2006.rx_id)
        motor_2006.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_pitch.rx_id)
        motor_pitch.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_yaw.rx_id)
        motor_yaw.read(can1.rx_data, stamp_ms);
    }
  }
}
