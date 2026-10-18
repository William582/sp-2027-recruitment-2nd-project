#ifndef SP__UART_HPP
#define SP__UART_HPP

#include <cstddef>
#include <cstdint>

#include "usart.h"

namespace sp
{
constexpr size_t UART_TX_BUF_LEN = 4;
constexpr size_t UART_RX_BUF_LEN = 16;

class GY56
{
public:
  GY56(UART_HandleTypeDef * huart);

  volatile float ir_distance_cm;     // 只读! recv()的输出
  volatile bool ir_valid;            // 只读! recv()的输出
  uint8_t tx_data[UART_TX_BUF_LEN];  // 只写! send()的输入
  uint8_t rx_data[UART_RX_BUF_LEN];  // recv()的DMA接收缓冲区

  // UART初始化配置：切换GY-56波特率到115200
  void config();

  // 启动DMA接收
  void start();

  // 接收并解析GY-56测距帧
  void recv(uint16_t size);

  // 发送GY-56命令帧（0xA5 + func + value + sum）
  void send(uint8_t func, uint8_t value);

private:
  UART_HandleTypeDef * huart_;
  uint8_t parse_buffer_[UART_RX_BUF_LEN];
  uint8_t parse_length_ = 0;

  // GY-56帧：0x5A 0x5A 0x15 0x03 [DH] [DL] [mode] [temp] [checksum]
  enum
  {
    FRAME_SIZE = 9,
    DIST_H_INDEX = 4,
    DIST_L_INDEX = 5,
    CHECKSUM_INDEX = 8
  };
};

}  // namespace sp
#endif  // SP__UART_HPP
