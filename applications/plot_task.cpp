#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

sp::Plotter plotter(&huart1, false);

extern sp::Mahony imu;
extern sp::DBus remote;

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  while (true) {
    plotter.plot(imu.roll, imu.pitch, imu.yaw);

    osDelay(10);
  }
}