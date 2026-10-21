/**
 * ============================================================
 * 红外测距任务（对应任务书 4.4.2.1）
 * ============================================================
 * 1. 独立 FreeRTOS 任务负责数据获取与更新
 * 2. 无效值 / 超量程 / 通信异常统一发布为 IR_INVALID_CM(0xFFFF)
 *    - 通信异常：LaserGY56::is_valid() 自带 500 ms 超时判据
 *    - 帧校验错误 / 状态位非 0：sp::LaserGY56 内部已经丢弃
 *    - 超量程或明显异常值：这里按有效窗口再过滤一次
 * 3. 结果放在 distance 里，供折叠任务直接读取
 * ============================================================
 */

#include "can/can.hpp"
#include "io/laser/laser.hpp"

sp::LaserGY56 laser(&huart6);

// 对外发布的测距值，单位 cm；IR_INVALID_CM 表示无效/超时/超量程
volatile uint16_t distance = IR_INVALID_CM;

// 绘图用副本：无效时给 -1，避免 0xFFFF 把曲线拉爆
volatile float plot_distance_cm = -1.0f;

// DBus 诊断：USART3 收到的事件次数与最后一次字节数
volatile uint16_t g_dbus_events = 0;
volatile uint16_t g_dbus_last_len = 0;

extern "C" void IR_ranging_task(void const * argument)
{
  (void)argument;

  laser.start_receive();
  laser.set_baud_rate_115200();
  laser.save_config();

  while (true) {
    uint16_t value = IR_INVALID_CM;

    if (laser.is_valid()) {
      const uint16_t raw = laser.get_distance();
      if ((raw >= IR_MIN_VALID_CM) && (raw <= IR_MAX_VALID_CM)) {
        value = raw;
      }
    }

    distance = value;
    plot_distance_cm = (value == IR_INVALID_CM) ? -1.0f : static_cast<float>(value);
    osDelay(1);
  }
}

// ---------------------------------
// 串口回调
// ---------------------------------
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  const uint32_t stamp_ms = osKernelSysTick();

  if (huart == &huart6) {
    laser.receive_complete(size);
  }
  else if (huart == &huart3) {
    g_dbus_events++;
    g_dbus_last_len = size;
    remote.update(size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart6) {
    laser.start_receive();
  }
  else if (huart == &huart3) {
    remote.request();
  }
}
