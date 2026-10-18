#ifndef SP__POWER_METER_HPP
#define SP__POWER_METER_HPP

#include <cstdint>

namespace sp
{
class PowerMeter
{
public:
  static constexpr uint16_t RX_ID = 0x301;  //功率计反馈帧id

  float current = 0.0f;  // 单位: A
  float voltage = 0.0f;  // 单位: V
  float power = 0.0f;    // 单位: W

  bool is_alive(uint32_t now_ms) const;
  void read(const uint8_t * data, uint32_t stamp_ms);

private:
  bool has_read_;
  uint32_t last_read_ms_ = 0;
};
}  // namespace sp

#endif  // SP__POWER_METER_HPP
