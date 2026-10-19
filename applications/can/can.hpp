/*该头文件

*/

#ifndef APPLICATIONS__CAN__CAN_HPP
#define APPLICATIONS__CAN__CAN_HPP

#include "can.h"
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

// IMU安装旋转矩阵（C板BMI088坐标系→机器人坐标系）
// 与mecanum_wheel一致，需根据实际安装方向调整
constexpr float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

// ============================================================
// 全局对象声明
// ============================================================
extern sp::DBus remote;
extern sp::CAN can1;
extern sp::RM_Motor motor_2006;
extern sp::RM_Motor motor_pitch;
extern sp::RM_Motor motor_yaw;
extern sp::BMI088 bmi088;
extern sp::Mahony imu;
extern sp::Plotter plotter;
extern sp::PID pid_position_2006;   // M2006 #1
extern sp::PID pid_position_pitch;  // M6020 #1
extern sp::PID pid_position_yaw;    // M6020 #2
// ============================================================
// 共享状态（跨任务）
// ============================================================

// 声明GY-56红外测距数据
extern uint16_t distance;  // 距离(cm)

// 折叠状态（用于云台折叠和轨迹跟踪任务）
enum class FoldState
{
  WAITING,
  FOLD,
  UNFOLD,
};

// ============================================================
// CAN通信函数声明
// ============================================================

// CAN初始化
void can_init();

// 发送全部云台电机命令（M2006 0x200帧 + GM6020 0x1FE帧）
void send_gimbal_commands(float torque_1, float torque_2, float torque_3);

// CAN接收回调
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan);

#endif  // APPLICATIONS__CAN__CAN_HPP