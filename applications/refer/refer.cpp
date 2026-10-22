#include "refer.hpp"

#include <stdint.h>

#include <cmath>

/*cpp中定义*/
//yaw定义:
void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;            // 1ms的freertos对应的滤波系数
  constexpr float threshold = 0.5f / 57.3f;  // 角度转换成弧度

  static uint8_t fire = 0;

  fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

//pitch定义:
void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
  if (fire_ratio == nullptr) {
    return;
  }
  constexpr float alpha = 0.001f;            // 1ms的freertos对应的滤波系数
  constexpr float threshold = 1.5f / 57.3f;  // 角度转换成弧度

  static uint8_t fire = 0;

  fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

  *fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

// /*调用*/
// yaw_detect(imu.yaw, plan_yaw, &yaw_ratio);    //imu.yaw是云台实际的yaw,plan_yaw是对应的期望值,以弧度为单位,yaw_ratio为自己定义的变量名
// pitch_detect(imu.pitch, plan_pitch, &pitch_ratio);    //imu.pitch是云台实际的pitch,plan_pitch是对应的期望值,以弧度为单位,pitch_ratio为自己定义的变量名
// 调用时需要放在一个 以1ms为周期freertos任务里