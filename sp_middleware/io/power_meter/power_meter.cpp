#include "power_meter.hpp"

namespace sp
{
bool PowerMeter::is_alive(uint32_t now_ms) const
{
  return has_read_ && (now_ms - last_read_ms_ < 100);
}

void PowerMeter::read(const uint8_t * data, uint32_t stamp_ms)
{
  // 更新时间戳
  last_read_ms_ = stamp_ms;

  const auto decode = [data](uint8_t high_index) {
    const uint16_t raw = (static_cast<uint16_t>(data[high_index]) << 8) | data[high_index + 1];
    return raw / 100.0f;
  };

  this->current = decode(0);
  this->voltage = decode(2);
  this->power = decode(4);

  has_read_ = true;
}
}  // namespace sp
