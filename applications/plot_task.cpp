#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "io/power_meter/power_meter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "referee/pm02/pm02.hpp"
#include "tim.h"
#include "tools/mahony/mahony.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

sp::Plotter plotter(&huart1, false);
//can通信
extern sp::CAN can1;
extern sp::PowerMeter power_meter;
//电机
//规定，从左上开始，按顺时针方向分别为电机1-4，减速比使用更新后的13.7,使用多圈计数
extern sp::RM_Motor motor_1;
extern sp::RM_Motor motor_2;
extern sp::RM_Motor motor_3;
extern sp::RM_Motor motor_4;
//麦轮
extern sp::Mecanum mecanum;
//dt7遥控器
extern sp::DBus remote;
//imu
extern sp::BMI088 bmi088;
extern sp::Mahony imu;
// referee
extern sp::PM02 pm02;
extern float total_power;  //总功率
extern "C" void plot_task(void const * argument)
{
  (void)argument;

  while (true) {
    plotter.plot(total_power, power_meter.power, 40.0f);

    osDelay(1);
  }
}
