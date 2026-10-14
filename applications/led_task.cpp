#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const * argument)
{
  (void)argument;
  led.start();
  uint8_t g = 10;
  uint8_t r = 10;
  uint8_t b = 10;
  while (true) {
    led.set(r * 0.01f, 0, 0);
    osDelay(100);
    led.set(0, g * 0.01f, 0);
    osDelay(100);
    led.set(0, 0, b * 0.01f);
    osDelay(100);
  }
}