#include "can/can.hpp"

// ============================================================
// 全局对象定义
// ============================================================
sp::CAN can1(&hcan1);
sp::DBus remote(&huart3);

// multi_circle = true：折叠轴要转 100°、pitch 轴经 36:1 后转子转好几圈，
// 必须用多圈累计角度，否则单圈角度会在 ±π（pitch 轴是 ±5°）处跳变。
//
// 注意：M2006 的 C610 实测 ID = 2（总线上只有 0x202，从无 0x201），
// 所以 motor_pitch 用 ID 2。若把电调拨回 ID 1，这里也要改回 1。
//
// yaw 用单圈角度（multi_circle=false）+ 角度环按最短路径算误差：
// yaw 的行程只有 ±22.5°，不需要多圈；单圈反而能避免多圈计数在 ±π 附近翻转
// 造成的 2π 跳变。pitch / fold 必须多圈，否则 ±5° / ±π 处会跳变。
sp::RM_Motor motor_yaw(2, sp::RM_Motors::GM6020, YAW_GEAR_RATIO, false);
sp::RM_Motor motor_pitch(2, sp::RM_Motors::M2006, PITCH_GEAR_RATIO, true);
sp::RM_Motor motor_fold(1, sp::RM_Motors::GM6020, FOLD_GEAR_RATIO, true);

// ============================================================
// 初始化
// ============================================================
void can_init()
{
  can1.config();
  can1.start();
}

void remote_init() { remote.request(); }

// ============================================================
// 拨杆读取（含调试旁路）
// ============================================================
sp::DBusSwitchMode remote_switch_r()
{
  if (DEBUG_ALLOW_NO_DBUS && !remote.is_open()) {
    return static_cast<sp::DBusSwitchMode>(DEBUG_FAKE_SW_R);
  }
  return remote.sw_r;
}

sp::DBusSwitchMode remote_switch_l()
{
  if (DEBUG_ALLOW_NO_DBUS && !remote.is_open()) {
    return static_cast<sp::DBusSwitchMode>(DEBUG_FAKE_SW_L);
  }
  return remote.sw_l;
}

// ============================================================
// 失能判据
// ============================================================
bool gimbal_is_disabled()
{
  // 失能模式具有最高控制优先级。
  // 分两类：
  //   1) 拨杆下档 —— 操作者的明确意图，立即失能
  //   2) 链路丢失（遥控器没数据 / 信号超时）—— 需要持续 LINK_LOST_MS 才算失能
  // 第 2 类加迟滞，是因为 DT7 偶尔丢几帧就会让 is_alive 变假；
  // 若立即判失能，折叠任务会重新回零、跟踪任务会重置相位，
  // 表现为"跟踪突然停一下、过一会儿又自己恢复"。
  if (remote_switch_r() == sp::DBusSwitchMode::DOWN) return true;
  if (remote_switch_l() == sp::DBusSwitchMode::DOWN) return true;

  static bool link_bad = false;
  static uint32_t link_bad_since = 0;

  const uint32_t now = osKernelSysTick();
  const bool link_ok = remote.is_open() && remote.is_alive(now);

  if (link_ok) {
    link_bad = false;
  }
  else {
    if (!link_bad) {
      link_bad = true;
      link_bad_since = now;
    }
    if ((now - link_bad_since) > LINK_LOST_MS) return true;
  }

  return false;
}

// ============================================================
// 发送云台电机命令
// ============================================================
void send_gimbal_commands(float torque_pitch, float torque_fold, float torque_yaw)
{
  // 二次保险：调用方即使算出了力矩，失能时也一律清零。
  if (gimbal_is_disabled()) {
    torque_pitch = 0.0f;
    torque_fold = 0.0f;
    torque_yaw = 0.0f;
  }

  // 0x200：M2006 pitch
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0;
  }
  motor_pitch.cmd(torque_pitch);
  motor_pitch.write(can1.tx_data);
  can1.send(motor_pitch.tx_id);

  // 0x1FE：GM6020 fold(ID1) + GM6020 yaw(ID2)
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; ++i) {
    can1.tx_data[i] = 0;
  }
  motor_fold.cmd(torque_fold);
  motor_yaw.cmd(torque_yaw);
  motor_fold.write(can1.tx_data);
  motor_yaw.write(can1.tx_data);
  can1.send(motor_fold.tx_id);
}
