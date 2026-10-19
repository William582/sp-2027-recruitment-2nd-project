#include "can/can.hpp"
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/laser/laser.hpp"

// ---------------------------------
// 定义外设
// ---------------------------------
sp::LaserGY56 laser(&huart1);
// 回调函数只能有一个，所以在此声明遥控器
extern sp::DBus remote;

// 定义GY-56红外测距数据
uint16_t distance = 0xFFFF;

extern "C" void IR_ranging_task(void const * argument)
{
  (void)argument;

  laser.start_receive();
  laser.set_baud_rate_115200();
  laser.save_config();

  while (true) {
    // 对外只发布未超时的有效距离, 否则发布无效值
    distance = laser.is_valid() ? laser.get_distance() : 0xFFFF;
    osDelay(1);
  }
}
//回调函数
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart1) {
    laser.receive_complete(size);
  }

  else if (huart == &huart3) {
    remote.update(size, stamp_ms);
    remote.request();
  }
}
// 错误回调
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart1) {
    laser.start_receive();
  }
  else if (huart == &huart3) {
    remote.request();
  }
}
