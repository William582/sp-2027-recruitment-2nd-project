/**
 * ============================================================
 * 绘图任务（对应任务书 4.4.8.4 / 4.4.2.1.3）
 * ============================================================
 * 通过 USART1 输出 10 路数据给 SerialPlot：
 *   ch1 yaw 目标     ch2 yaw 实际
 *   ch3 pitch 目标   ch4 pitch 实际
 *   ch5 yaw 力矩     ch6 pitch 力矩
 *   ch7 测距结果(cm，无效时为 -1)
 *   ch8 折叠轴角度(rad)  ch9 折叠轴力矩(N·m)
 *   ch10 电机在线位图（bit0=yaw, bit1=pitch, bit2=fold）
 *   ch11 CAN 反馈 ID 位图（bit0=0x201 ... bit5=0x206，100 ms 窗口）
 * 200 Hz 输出，足够分辨 680 ms 的轨迹周期。
 * ============================================================
 */

#include "FreeRTOS.h"
#include "task.h"

#include "can/can.hpp"

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
      plot_yaw_target, plot_yaw_actual, plot_pitch_target, plot_pitch_actual, plot_yaw_torque,
      plot_pitch_torque, plot_distance_cm, plot_fold_angle, plot_fold_torque, plot_motor_status,
      plot_can_id_mask, plot_rc_status, plot_fold_state, plot_dbus_len, plot_yaw_speed,
      plot_pitch_speed, plot_imu_pitch);

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PLOT_PERIOD_MS));
  }
}
