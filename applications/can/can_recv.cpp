#include "can/can.hpp"

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    can1.recv();

    if (can1.frame_type) continue;  // 跳过扩展帧

    // 分发到对应电机
    if (can1.rx_id == motor_pitch.rx_id)
      motor_pitch.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motor_fold.rx_id)
      motor_fold.read(can1.rx_data, stamp_ms);
    else if (can1.rx_id == motor_yaw.rx_id)
      motor_yaw.read(can1.rx_data, stamp_ms);
  }
}