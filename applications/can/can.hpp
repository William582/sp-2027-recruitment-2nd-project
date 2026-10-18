#ifndef APPLICATIONS__CAN__CAN_HPP
#define APPLICATIONS__CAN__CAN_HPP

#include "can.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "spi.h"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "usart.h"

// ============================================================
// 全局外设对象
// ============================================================

// CAN总线
extern sp::CAN can1;

// 电机定义（共3个）：
// motor_pitch : M2006 ID=1, 云台顶端, 折叠+pitch跟随（共享）
// motor_fold  : GM6020 ID=1, 云台中部, 折叠用
// motor_yaw   : GM6020 ID=2, 云台底部, yaw跟随
extern sp::RM_Motor motor_pitch;  // M2006 #1
extern sp::RM_Motor motor_fold;   // GM6020 #1
extern sp::RM_Motor motor_yaw;    // GM6020 #2

// IMU
extern sp::BMI088 bmi088;
extern sp::Mahony imu;

// Plotter (UART1, 不用DMA)
extern sp::Plotter plotter;

// ============================================================
// 共享状态（跨任务）
// ============================================================

// 云台工作模式
//   0 = IDLE        空闲，未折叠，未跟随
//   1 = FOLDING     折叠/展开中，折叠任务控制电机
//   2 = FOLDED      已折叠
//   3 = TRACKING    轨迹跟随中，跟随任务控制电机
extern "C" volatile int gimbal_mode;

// GY-56红外测距数据
extern "C" volatile float ir_distance_cm;  // 距离(cm)
extern "C" volatile bool ir_valid;         // 数据有效

// ============================================================
// 函数声明
// ============================================================

// CAN初始化 + 电机CAN配置
extern void can_init();

// 发送全部云台电机命令（M2006 0x200帧 + GM6020 0x1FE帧）
extern void can_send_gimbal();

// CAN接收回调
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan);

#endif  // APPLICATIONS__CAN__CAN_HPP