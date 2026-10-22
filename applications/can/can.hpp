#ifndef APPLICATIONS__CAN__CAN_HPP
#define APPLICATIONS__CAN__CAN_HPP

#include <cstdint>

#include "can.h"
#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"

// ============================================================
// 机械参数
// ============================================================
// 轴映射（对接线，改之前先确认电调/CAN 接线）：
//   motor_yaw    GM6020  ID2   底座 yaw 轴
//   motor_pitch  M2006   ID1   pitch 轴（末端红外测距模块）
//   motor_fold   GM6020  ID1   中部折叠轴
//
// 注意：RM_Motor 的 ratio 语义是「电机到关节的总减速比」，
// rm_motor.cpp 内部对 M2006 已经除过 36，所以这里必须填实际总减速比。
constexpr float YAW_GEAR_RATIO = 1.0f;     // GM6020 直驱
constexpr float PITCH_GEAR_RATIO = 36.0f;  // M2006 自带 36:1，同步带 1:1
constexpr float FOLD_GEAR_RATIO = 1.0f;    // GM6020 直驱

constexpr float PI_F = 3.14159265358979f;

// 折叠 / 展开行程：中部 GM6020 旋转 130°
constexpr float FOLD_ANGLE_RAD = 130.0f * PI_F / 180.0f;

// 展开方向：上电时为折叠位，展开 = 角度 +100°。
// 若实机展开方向相反（向下折），把这里改成 -1.0f 即可。
constexpr float FOLD_UNFOLD_SIGN = 1.0f;

// ============================================================
// 控制周期与公共限幅
// ============================================================
constexpr float CONTROL_DT = 0.001f;         // 1 kHz 控制周期
constexpr uint32_t FEEDBACK_WAIT_MS = 1000;  // 上电等待电机反馈的最长时间

// 输出限幅（按电机峰值留余量，防止过流/机构冲击）
constexpr float YAW_MAX_TORQUE = 2.0f;    // GM6020 峰值约 2.22 N·m
constexpr float PITCH_MAX_TORQUE = 1.8f;  // M2006 P36 峰值约 1.8 N·m
constexpr float FOLD_MAX_TORQUE = 1.8f;   // GM6020 峰值约 2.22 N·m

constexpr float YAW_MAX_VEL = 9.0f;  // 位置环输出（目标速度）限幅 rad/s
constexpr float PITCH_MAX_VEL = 8.0f;

// 前馈增益：速度前馈取 1.0，加速度前馈量纲等价于转动惯量，需实测微调
// 跟踪调试初期取偏保守的值，优先保证稳定，再逐步加大
// 实测 yaw 实际曲线超前目标约 40 ms（相位超前约 21°），说明前馈过驱，先各降一档
constexpr float YAW_FF_VEL = 1.0f;
constexpr float YAW_FF_ACC = 0.0012f;
constexpr float PITCH_FF_VEL = 1.0f;
constexpr float PITCH_FF_ACC = 0.002f;

// ============================================================
// 自动折叠参数（4.4.7）
// ============================================================
constexpr float FOLD_TRIGGER_CM = 30.0f;           // 障碍物触发距离
constexpr uint16_t FOLD_CONFIRM_TICKS = 50;        // 连续 50 个周期满足才动作（消抖）
constexpr float FOLD_REACH_TOLERANCE_RAD = 0.03f;  // 到位判据
constexpr uint32_t FOLD_TIMEOUT_MS = 3000;         // 折叠/展开超时
constexpr float FOLD_LIMIT_MARGIN_RAD = 0.50f;     // 行程越限保护余量
constexpr float FOLD_MAX_CMD_RATE = 1.5f;          // 折叠指令斜坡限速 rad/s（100°约 1.2 s）

// 锁定"折叠基准"的条件：折叠轴必须连续静止一段时间。
// （现在基准由回零 HOMING 建立，这两个常量仅保留给手动标定参考）
constexpr float FOLD_REF_STILL_RAD_S = 0.20f;   // 静止判据 rad/s
constexpr uint16_t FOLD_REF_STILL_TICKS = 300;  // 连续静止 300 ms

// 回零（homing）：使能后先朝折叠方向低力矩压到机械限位，停稳后以此为零点。
// 这样不管上电/复位时机械臂停在哪里，展开行程都不会偏。
constexpr float FOLD_HOME_TORQUE = 0.45f;         // 回零力矩（朝折叠方向，取负号）
constexpr float FOLD_HOME_STILL_RAD_S = 0.06f;    // （参考）判定"顶到限位"的速度阈值
constexpr float FOLD_HOME_MOVE_EPS_RAD = 0.003f;  // 回零时判定"没动"的角度变化阈值
constexpr uint16_t FOLD_HOME_STILL_TICKS = 300;   // 连续 300 ms 不动即认为到限位
constexpr uint32_t FOLD_HOME_TIMEOUT_MS = 8000;   // 回零总超时

// 折叠前先把末端 M2006 收到折叠位，等它到位才允许中部折叠（防干涉）
constexpr uint32_t FOLD_STOW_TIMEOUT_MS = 2500;

// 折叠轴速度阻尼：PID 的 D 项在 1 ms 周期下会把编码器噪声放大约 1000 倍，
// 所以不用 kd，改成"滤波后的速度 × 阻尼系数"直接减到力矩上。
constexpr float FOLD_DAMPING = 0.15f;          // N·m per rad/s
constexpr float FOLD_SPEED_LPF_ALPHA = 0.10f;  // 速度低通

// 折叠触发源：
//   true  = 遥控器拨轮 ch_lu > FOLD_TRIGGER_CH_LU 触发（测距模块缺失期间的临时方案）
//   false = GY-56 测距 < 30cm 且 IMU 静止 触发（任务书 4.4.7 的正式方案）
// 测距模块接回后把这里改成 false 即可，其余状态机不用动。
constexpr bool FOLD_TRIGGER_USE_WHEEL = true;
constexpr float FOLD_TRIGGER_CH_LU = 0.5f;         // 拨轮触发阈值
constexpr uint16_t FOLD_WHEEL_CONFIRM_TICKS = 20;  // 拨轮触发消抖周期数（20 ms）

constexpr float IMU_STATIONARY_RAD_S = 0.05f;  // 静止判据阈值
constexpr uint16_t IMU_SETTLE_TICKS = 200;     // 上电后 IMU 稳定计数

// 红外测距有效窗口（无效 / 超量程统一发布为 IR_INVALID_CM）
constexpr uint16_t IR_MIN_VALID_CM = 3;
constexpr uint16_t IR_MAX_VALID_CM = 3000;
constexpr uint16_t IR_INVALID_CM = 0xFFFF;

// ============================================================
// 全局对象
// ============================================================
// IMU 安装旋转矩阵（C 板 BMI088 坐标系 → 机器人坐标系）
constexpr float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

extern sp::DBus remote;
extern sp::CAN can1;

extern sp::RM_Motor motor_yaw;    // GM6020 ID2  底座 yaw
extern sp::RM_Motor motor_pitch;  // M2006  ID1  末端 pitch
extern sp::RM_Motor motor_fold;   // GM6020 ID1  中部折叠

extern sp::BMI088 bmi088;
extern sp::Mahony imu;
extern sp::Plotter plotter;

// ============================================================
// PID
// ============================================================
extern sp::PID pid_position_yaw;
extern sp::PID pid_speed_yaw;
extern sp::PID pid_position_pitch;  // M2006 位置环
extern sp::PID pid_speed_pitch;     // M2006 速度环
extern sp::PID pid_fold;            // 折叠轴：位置 -> 力矩

// ============================================================
// 折叠状态机
// ============================================================
enum class FoldState : uint8_t
{
  DISABLED,   // 失能（右拨杆下档 / 左拨杆下档 / 遥控器链路丢失）
  HOMING,     // 回零：朝折叠方向低力矩压到机械限位
  UNFOLDING,  // 正在展开
  WAITING,    // 已展开，测距工作，等待障碍物
  FOLDING,    // 正在折叠
  FOLDED,     // 已折叠，等待左拨杆上档重新展开
  FAULT,      // 超时 / 越限等异常，已退出自动流程
  STOWING,    // 折叠前先把末端 M2006 收到折叠位（中部仍保持展开，防干涉）
};

extern volatile FoldState fold_state;
extern volatile float fold_torque;  // 折叠任务输出，由轨迹任务打包进 0x1FE

// ============================================================
// 共享传感器数据
// ============================================================
extern volatile uint16_t distance;       // 单位 cm，IR_INVALID_CM 表示无效
extern volatile float plot_distance_cm;  // 绘图用：无效时为 -1
extern volatile bool imu_stationary;     // IMU 静止标志

// ============================================================
// 绘图通道（SerialPlot）
//   ch1 yaw目标  ch2 yaw实际  ch3 pitch目标  ch4 pitch实际
//   ch5 yaw力矩  ch6 pitch力矩 ch7 测距(cm)
//   ch8 折叠轴角度(rad)  ch9 折叠轴力矩(N·m)  ch10 电机在线位图
//   ch10 位定义：bit0 = yaw 在线, bit1 = pitch 在线, bit2 = fold 在线
// ============================================================
extern volatile float plot_yaw_target;
extern volatile float plot_yaw_actual;
extern volatile float plot_pitch_target;
extern volatile float plot_pitch_actual;
extern volatile float plot_yaw_torque;
extern volatile float plot_pitch_torque;
extern volatile float plot_fold_angle;
extern volatile float plot_fold_torque;
extern volatile float plot_motor_status;

// CAN 反馈 ID 位图（诊断用）：bit0=0x201 ... bit5=0x206，100 ms 清零一次
extern volatile uint8_t g_can_id_mask;
extern volatile float plot_can_id_mask;

// 遥控器诊断：open*100 + sw_r*10 + sw_l（0 表示根本没收到 DBus 帧）
extern volatile float plot_rc_status;
// DBus 串口诊断：0 = USART3 从无事件；1000+len = 收到过 len 字节的帧（正常应为 1018）
extern volatile float plot_dbus_len;
// 折叠状态机当前状态（0=DISABLED 1=HOMING 2=UNFOLDING 3=WAITING 4=FOLDING 5=FOLDED 6=FAULT）
extern volatile float plot_fold_state;
// 速度诊断：yaw / pitch 的编码器速度（rad/s）
extern volatile float plot_yaw_speed;
extern volatile float plot_pitch_speed;
// 末端 IMU 的 pitch（rad）：C 板固定在末端，这就是末端绝对倾角，0 = 水平
extern volatile float plot_imu_pitch;

// ============================================================
// 调试旁路（验收前必须关闭）
// ============================================================
// 接收机未连接时，用下面的"假拨杆状态"代替遥控器输入，
// 便于在没有遥控器的情况下验证机构动作（展开 / 折叠 / 轨迹）。
// 设为 false 后，接收机没数据就一律失能 —— 这是验收要求的行为。
// 【当前为 false】云台所有控制必须来自遥控器，没有 DBus 数据就不动作。
constexpr bool DEBUG_ALLOW_NO_DBUS = false;
constexpr int DEBUG_FAKE_SW_R = 1;  // 0=DOWN 1=MID 2=UP
constexpr int DEBUG_FAKE_SW_L = 1;  // 0=全失能 1=展开 2=轨迹跟踪（仅旁路打开时生效）

// 调试用：非 0 时折叠轴直接输出该恒定力矩，绕开状态机与 PID。
// 用来单独验证"电机能不能把机构推动"，排查传动/力矩问题。正常必须为 0。
constexpr float DEBUG_FOLD_OPEN_LOOP_TORQUE = 0.0f;

sp::DBusSwitchMode remote_switch_r();
sp::DBusSwitchMode remote_switch_l();

// DBus 串口诊断（由 IR 任务的 UART 回调维护）
extern volatile uint16_t g_dbus_events;
extern volatile uint16_t g_dbus_last_len;

// ============================================================
// 跟踪零点（实测写死，不再每次任务开始时重新标定）
// ============================================================
// 2026-10-21 实测：展开完成、未跟踪、人工摆到水平时
//   M2006 关节角 = -0.6548 rad
//   末端 IMU 倾角 = -0.0361 rad
// 这样初始化完全不动 pitch，展开完成后直接 READY。
constexpr float PITCH_ZERO_JOINT = -0.6548f;  // 水平姿态下的 M2006 关节角（rad）
constexpr float IMU_ZERO_PITCH = -0.0361f;    // 同一姿态下的 IMU 倾角（rad，用于校验）
constexpr float PITCH_SIGN = 1.0f;            // 轨迹 pitch 叠加到关节角的符号
// 认为"已找平 / 可以起摆"的判据：关节角距离水平零点小于此值（约 2.9°）
constexpr float PITCH_READY_TOL = 0.05f;

// 折叠状态下 M2006 的目标角度（实测写死）。
// 折叠时末端收在机构内侧，2026-10-21 实测自然静止角约 -0.047 rad，
// 用这个值作为折叠位的保持目标，避免它乱摆。
constexpr float PITCH_FOLD_JOINT = -0.0470f;

// 链路丢失（遥控器/电机反馈）判定为失能的迟滞时间。
// 拨杆下档是操作者明确意图，仍然立即生效；只有"链路抖动"才走这个迟滞，
// 避免 DT7 丢几帧就把整个任务重置、出现"跟踪停一下又恢复"。
constexpr uint32_t LINK_LOST_MS = 250;

// ============================================================
// 接口
// ============================================================
// 失能判据：遥控器未连接 / 信号丢失 / 右拨杆下档 / 左拨杆下档
bool gimbal_is_disabled();

void can_init();
void remote_init();

// 发送全部云台电机命令。
// 参数顺序：M2006 pitch 力矩、GM6020 fold 力矩、GM6020 yaw 力矩（单位 N·m）。
// 内部会再判一次失能，失能时强制清零，保证失能优先级最高。
void send_gimbal_commands(float torque_pitch, float torque_fold, float torque_yaw);

void clear_tracking_pids();
void clear_fold_pids();

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan);

#endif  // APPLICATIONS__CAN__CAN_HPP
