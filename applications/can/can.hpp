#ifndef APPLICATIONS__CAN__CAN_HPP
#define APPLICATIONS__CAN__CAN_HPP

#include "can.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "spi.h"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "usart.h"

// IMU安装旋转矩阵（C板BMI088坐标系→机器人坐标系）
// 与mecanum_wheel一致，需根据实际安装方向调整
constexpr float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// ============================================================
// 全局对象定义
// ============================================================

extern sp::DBus remote;
extern sp::CAN can1;
extern sp::RM_Motor motor_2006;
extern sp::RM_Motor motor_fold;
extern sp::RM_Motor motor_yaw;
extern sp::BMI088 bmi088;
extern sp::Mahony imu;
extern sp::Plotter plotter;

// ============================================================
// 共享状态（跨任务）
// ============================================================

// GY-56红外测距数据
extern "C" volatile float distance;  // 距离(cm)

// ============================================================
// 函数声明
// ============================================================

// CAN初始化 + 电机CAN配置
extern void can_init();

// 发送全部云台电机命令（M2006 0x200帧 + GM6020 0x1FE帧）
extern "C" void send_gimbal_commands(float torque_1, float torque_2, float torque_3);

// CAN接收回调
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan);

#endif  // APPLICATIONS__CAN__CAN_HPP