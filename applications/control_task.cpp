#include "cmsis_os.h"
#include "tim.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"
// ============================================================
//1.调用外设
// ============================================================
//can通信
sp::CAN can1(&hcan1);
//电机
//规定，从左上开始，按顺时针方向分别为电机1-4
sp::RM_Motor motor_1(1, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_2(2, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_3(3, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_4(4, sp::RM_Motors::M3508, sp::M3508_P19);
// 麦克纳姆轮底盘运动学，尺寸参数暂置为 0，使用前需设置为实际值
//底盘基本参数
static float WHEEL_RADIUS = 0.076f;  // 轮子半径，单位：m
static float HALF_LENGTH = 0.2f;     // 机器人前后轮中心距的一半，单位：m
static float HALF_WIDTH = 0.2f;      // 机器人左右轮中心距的一半，单位：m
sp::Mecanum mecanum(WHEEL_RADIUS, HALF_LENGTH, HALF_WIDTH, false, false, true, true);
//dt7遥控器
sp::DBus remote(&huart3);
//imu
const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);
// ============================================================
//2.imu初始化
// ============================================================
bool imu_ready = false;
uint8_t first_temperate;
// -------------------- 控制参数 --------------------
// IMU温度
constexpr float IMU_TEMP = 50.0f;
// PID参数
constexpr float IMU_TEMP_KP = 400.0f;  // 根据实际发热丝的功率微调这个KP
constexpr float IMU_TEMP_KI = 0.0f;
constexpr float IMU_TEMP_KD = 0.0f;
constexpr float IMU_TEMP_MAXOUT = 2000.0f;  // 建议限制 PID 最大输出，防止静态积分时全功率烘烤
constexpr float IMU_TEMP_MAXIOUT = 0.0f;

sp::PID imu_temp_pid(
  1e-3, IMU_TEMP_KP, IMU_TEMP_KI, IMU_TEMP_KD, IMU_TEMP_MAXOUT, IMU_TEMP_MAXIOUT, 1.0f);
//imu初始化函数
void imu_init()

{
  bmi088.init();
  imu_ready = true;
}
//imu数据更新与姿态解算函数
void imu_update()
{
  if (!imu_ready) {
    return;
  }
  bmi088.update();
  imu.update(bmi088.acc, bmi088.gyro);
}
// 陀螺仪温度控制函数
void imu_temp_control(float temp)
{
  uint16_t tempPWM = 0;

  if (IMU_TEMP - temp > 10.0f) {
    tempPWM = 4000;
  }
  else {
    imu_temp_pid.calc(IMU_TEMP, temp);

    // 限制输出下限，防止输出负数导致 PWM 溢出异常
    if (imu_temp_pid.out < 0.0f) {
      imu_temp_pid.out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid.out;
  }

  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, tempPWM);
}
// ============================================================
//3.dt7遥控器初始化
// ============================================================
//遥控器初始化函数
void remote_init() { remote.request(); }
// 遥控器接收回调函数
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  if (huart == &huart3) {
    remote.update(size, osKernelSysTick());
    remote.request();
  }
}
// 遥控器错误回调函数
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}
// ============================================================
// 4.can通信
// ============================================================
constexpr size_t MOTOR_COUNT = 4;
//向电机发送控制命令
void send_motor_commands(float torque_1, float torque_2, float torque_3, float torque_4)
{
  const float torques[MOTOR_COUNT] = {torque_1, torque_2, torque_3, torque_4};
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0;
  }

  motor_1.cmd(torques[0]);
  motor_1.write(can1.tx_data);

  motor_2.cmd(torques[1]);
  motor_2.write(can1.tx_data);

  motor_3.cmd(torques[2]);
  motor_3.write(can1.tx_data);

  motor_4.cmd(torques[3]);
  motor_4.write(can1.tx_data);

  can1.send(motor_1.tx_id);  // 四个 M3508 共用 0x200 控制帧，统一发送
}
// 读取电机反馈
void read_motor_feedback(uint32_t stamp_ms)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    can1.recv(CAN_RX_FIFO0);

    if (can1.rx_id == motor_1.rx_id) {
      motor_1.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_2.rx_id) {
      motor_2.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_3.rx_id) {
      motor_3.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_4.rx_id) {
      motor_4.read(can1.rx_data, stamp_ms);
    }
  }
}
// ============================================================
//5.PID
// ============================================================
constexpr float MOTOR_PID_DT = 0.001f;

struct PIDParameters
{
  PIDParameters(float p, float i, float d, float mo, float mi, float a)
  : kp(p), ki(i), kd(d), max_out(mo), max_iout(mi), alpha(a)
  {
  }
  float kp;
  float ki;
  float kd;
  float max_out;
  float max_iout;
  float alpha;
};
// 左上电机
PIDParameters motor_1_speed_pid_para(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
// 右上电机
PIDParameters motor_2_speed_pid_para(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
// 右下电机
PIDParameters motor_3_speed_pid_para(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
// 左下电机
PIDParameters motor_4_speed_pid_para(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
// imu PID(用于控制电机方向，消除误差)
PIDParameters imu_position_pid_para(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
// PID实例化（针对速度环使用动态积分）
sp::PID motor_1_speed_pid(
  MOTOR_PID_DT, motor_1_speed_pid_para.kp, motor_1_speed_pid_para.ki, motor_1_speed_pid_para.kd,
  motor_1_speed_pid_para.max_out, motor_1_speed_pid_para.max_iout, motor_1_speed_pid_para.alpha,
  false, true);

sp::PID motor_2_speed_pid(
  MOTOR_PID_DT, motor_2_speed_pid_para.kp, motor_2_speed_pid_para.ki, motor_2_speed_pid_para.kd,
  motor_2_speed_pid_para.max_out, motor_2_speed_pid_para.max_iout, motor_2_speed_pid_para.alpha,
  false, true);

sp::PID motor_3_speed_pid(
  MOTOR_PID_DT, motor_3_speed_pid_para.kp, motor_3_speed_pid_para.ki, motor_3_speed_pid_para.kd,
  motor_3_speed_pid_para.max_out, motor_3_speed_pid_para.max_iout, motor_3_speed_pid_para.alpha,
  false, true);

sp::PID motor_4_speed_pid(
  MOTOR_PID_DT, motor_4_speed_pid_para.kp, motor_4_speed_pid_para.ki, motor_4_speed_pid_para.kd,
  motor_4_speed_pid_para.max_out, motor_4_speed_pid_para.max_iout, motor_4_speed_pid_para.alpha,
  false, true);

sp::PID imu_position_pid(
  MOTOR_PID_DT, imu_position_pid_para.kp, imu_position_pid_para.ki, imu_position_pid_para.kd,
  imu_position_pid_para.max_out, imu_position_pid_para.max_iout, imu_position_pid_para.alpha, true,
  true);  //采用角度控制
// 双环控制函数
float dual_loop_control(
  sp::PID & position_pid, sp::PID & speed_pid, float target_angle, float actual_angle,
  float actual_speed)
{
  position_pid.calc(target_angle, actual_angle);
  const float target_speed = position_pid.out;
  // 位置外环输出目标速度；速度内环输出最终控制量
  speed_pid.calc(target_speed, actual_speed);
  return speed_pid.out;
}
// 单环控制函数
float single_loop_control(sp::PID & pid, float target_variable, float actual_variable)
{
  pid.calc(target_variable, actual_variable);
  return pid.out;
}
// 清除全部 PID 状态
void clear_control_pids()
{
  // 清除 PID 状态(imu位置环)
  imu_position_pid.clear();
  // 清除 PID 状态(电机速度环)
  motor_1_speed_pid.clear();
  motor_2_speed_pid.clear();
  motor_3_speed_pid.clear();
  motor_4_speed_pid.clear();
}
// ============================================================
//6.电机编码器初始化
// ============================================================
// 记录单个轮子相对于初始位置累计转过的角度，用于判断走过的距离，单位：rad
float record_wheel_angle(float current_angle, float & initial_angle, bool & initialized)
{
  if (!initialized) {
    initial_angle = current_angle;
    initialized = true;
  }

  return current_angle - initial_angle;
}

extern "C" void control_task(void const * argument)
{
  (void)argument;
  imu_init();
  remote_init();
  //启动定时器10，用于IMU加热
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

  // 电机输出力矩
  float torque_1 = 0.0f;
  float torque_2 = 0.0f;
  float torque_3 = 0.0f;
  float torque_4 = 0.0f;

  //限制参数
  const float MAX_TORQUE = 2000.0f;  // 电机最大输出扭矩，单位：mN·m
  const float MAX_SPEED_X = 1.0f;    // 机器人在x方向上的最大速度，单位：m/s
  const float MAX_SPEED_Y = 1.0f;    // 机器人在y方向上的最大速度，单位：m/s
  const float MAX_SPEED_W = 1.0f;    // 机器人绕z轴的额定角速度，单位：rad/s

  while (true) {
    imu_update();
    read_motor_feedback(osKernelSysTick());
    imu_temp_control(bmi088.temp);

    const sp::DBusSwitchMode right_switch = remote.sw_r;  //获取右拨杆档位
    const float left_stick_vertical = remote.ch_lv;
    const float left_stick_horizontal = remote.ch_lh;  //获取左摇杆水平和垂直方向的值
    const float left_dial = remote.ch_lu;              //获取左拨轮的值
    switch (right_switch) {
      // ----------------------------------------------------
      // DOWN：失能模式
      // ----------------------------------------------------
      case sp::DBusSwitchMode::DOWN: {
        torque_1 = 0.0f;
        torque_2 = 0.0f;
        torque_3 = 0.0f;
        torque_4 = 0.0f;

        clear_control_pids();

        break;
      }
      // ----------------------------------------------------
      // MID：全向移动模式
      // ----------------------------------------------------
      case sp::DBusSwitchMode::MID: {
        constexpr float DIAL_DEADZONE = 0.05f;
        float target_wz = 0.0f;
        if (left_dial > DIAL_DEADZONE) {
          target_wz = MAX_SPEED_W;
        }
        else if (left_dial < -DIAL_DEADZONE) {
          target_wz = -MAX_SPEED_W;
        }
        else {
          target_wz = 0.0f;
        }  //判断语句来获取左拨轮的方向，并加上了一个小的死区，以避免拨轮在中间位置时产生微小的误差。

        // 底盘速度 -> 各轮转速
        // vx: 前进速度, 单位: m/s ； vy: 左移速度, 单位: m/s ； wz: 大拇指朝上，右手螺旋方向转速, 单位: rad/s(右手系)
        //根据左摇杆的输入，计算出底盘的速度，并将其转换为各轮的转速
        mecanum.calc(
          MAX_SPEED_X * left_stick_vertical, MAX_SPEED_Y * left_stick_horizontal, target_wz);

        // 车轮速度环：目标角速度与电机反馈角速度均为减速箱输出轴 rad/s,无需进行减速比换算
        torque_1 = single_loop_control(motor_1_speed_pid, mecanum.speed_lf, motor_1.speed);
        torque_2 = single_loop_control(motor_2_speed_pid, mecanum.speed_rf, motor_2.speed);
        torque_3 = single_loop_control(motor_3_speed_pid, mecanum.speed_rr, motor_3.speed);
        torque_4 = single_loop_control(motor_4_speed_pid, mecanum.speed_lr, motor_4.speed);

        // 实际车轮速度 -> 更新底盘实际速度（可用于监控或后续控制）
        //这里电机编号遵从中间件左上、右上、右下、左下的顺序排列的，所以在调用update函数时，传入的参数顺序应该是motor_1.speed, motor_4.speed, motor_2.speed, motor_3.speed
        mecanum.update(motor_1.speed, motor_4.speed, motor_2.speed, motor_3.speed);
        break;
      }

      // ----------------------------------------------------
      // UP：自动直线行驶模式
      // ----------------------------------------------------
      case sp::DBusSwitchMode::UP: {
        break;
      }

      default: {
        torque_1 = 0.0f;
        torque_2 = 0.0f;
        torque_3 = 0.0f;
        torque_4 = 0.0f;
        clear_control_pids();
        break;
      }
    }

    // ----------------------------------------------------
    // 10. 发送电机控制指令
    // ----------------------------------------------------

    send_motor_commands(torque_1, torque_2, torque_3, torque_4);

    osDelay(1);
  }
}
