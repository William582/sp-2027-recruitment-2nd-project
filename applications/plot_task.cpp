/**
 * ============================================================
 * 绘图任务（对应任务书 4.4.8.4 / 4.4.2.1.3）
 * ============================================================
 * 通过 USART1 输出 17 路数据给 SerialPlot：
 *   ch1  distance 滤波后(cm)     ch2  distance 原始值(cm，无效 -1)
 *   ch3  pitch 目标              ch4  pitch 实际
 *   ch5  yaw 力矩                ch6  pitch 力矩
 *   ch7  测距滤波后(cm，无效 -1)
 *   ch8  折叠轴角度(rad)         ch9  折叠轴力矩(N·m)
 *   ch10 电机在线位图
 *   ch11 CAN 反馈 ID 位图（bit0=0x201 ... bit5=0x206，100 ms 窗口）
 *   ch12 yaw 对准度              ch13 pitch 对准度
 *   ch14 DBus 长度               ch15 yaw 速度
 *   ch16 pitch 速度              ch17 IMU pitch
 * 200 Hz 输出，足够分辨 680 ms 的轨迹周期。
 * ============================================================
 */

#include "FreeRTOS.h"
#include "can/can.hpp"
#include "task.h"

sp::Plotter plotter(&huart1, false);

namespace
{
constexpr uint32_t PLOT_PERIOD_MS = 5;  // 200 Hz
}

extern "C" void plot_task(void const * argument)
{
  (void)argument;

  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    plotter.plot(
      distance, plot_distance_raw, plot_pitch_target, plot_pitch_actual, plot_yaw_torque,
      plot_pitch_torque, plot_distance_cm, plot_fold_angle, plot_fold_torque, plot_motor_status,
      plot_can_id_mask, plot_yaw_fire, plot_pitch_fire, plot_dbus_len, plot_yaw_speed,
      plot_pitch_speed, plot_imu_pitch);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PLOT_PERIOD_MS));
  }
}
