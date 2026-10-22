#ifndef SP_REFER_HPP
#define SP_REFER_HPP

/*hpp中声明*/
void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio);

#endif  // SP_REFER_HPP