#include "can/can.hpp"

// 最近 100 ms 内出现过的电机反馈 ID 位图：
// bit0 = 0x201, bit1 = 0x202, ... bit5 = 0x206
// 用来诊断"电机到底有没有在总线上发帧"以及 C610 的 ID 设成了几。
volatile uint8_t g_can_id_mask = 0;

// CAN 接收回调（由 CAN1_RX0 中断调用）。
// 任务里不再手动调用本函数，避免任务上下文和中断上下文并发写同一个 motor 对象。
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  const uint32_t stamp_ms = osKernelSysTick();
  CAN_RxHeaderTypeDef header;
  uint8_t data[sp::CAN_DATA_LEN];

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    // 非 CAN1 的帧（例如功率计）只取走数据，避免 FIFO 满导致死循环
    if (hcan != &hcan1) {
      HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data);
      continue;
    }

    can1.recv();

    // 记录 0x201~0x206 的到达情况（不区分是不是我们关心的那三台）
    if ((can1.rx_id >= 0x201) && (can1.rx_id <= 0x206)) {
      g_can_id_mask |= static_cast<uint8_t>(1U << (can1.rx_id - 0x201));
    }

    if (can1.rx_id == motor_pitch.rx_id) {
      motor_pitch.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_fold.rx_id) {
      motor_fold.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_yaw.rx_id) {
      motor_yaw.read(can1.rx_data, stamp_ms);
    }
  }
}
