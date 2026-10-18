#include "can/can.hpp"
#include "uart/uart_task.hpp"

// 红外测距FreeRTOS任务
// 职责：初始化GY-56(UART6)，持续接收距离数据供折叠任务使用
extern "C" void IR_ranging_task(void const * argument)
{
  (void)argument;

  // 初始化UART6 + GY-56（含波特率切换）
  gy56_sensor.init();

  while (true) {
    // GY-56为连续输出模式，数据通过DMA中断自动更新 ir_distance_cm
    // 此任务仅做心跳/超时检测
    // 如果超过500ms未收到有效数据，标记无效
    static uint32_t last_valid_tick = 0;

    if (ir_valid) {
      last_valid_tick = osKernelSysTick();
    }

    if (osKernelSysTick() - last_valid_tick > 500) {
      ir_valid = false;
    }

    osDelay(50);
  }
}