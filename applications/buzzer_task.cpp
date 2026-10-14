#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

extern "C" void buzzer_task(void const * argument)
{
  (void)argument;

  for (int i = 0; i <= 3; i++) {
    buzzer.set(i * 1000 + 500, 0.1f);
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }

  while (true) {
    osDelay(100);
  }
}