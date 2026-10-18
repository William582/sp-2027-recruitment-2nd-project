#include "uart/uart_task.hpp"

#include <cstring>

#include "cmsis_os.h"

// 参考文档：https://tj-superpower.feishu.cn/file/SmGebL87YovSHDxABFQcm1J4ngy
namespace sp
{
GY56::GY56(UART_HandleTypeDef * huart) : huart_(huart) {}

// ============================================================
// UART初始化配置
// 步骤：先以9600通信将GY-56切换到115200，再以115200重新初始化
// ============================================================
void GY56::config()
{
  // --- 以9600bps发送波特率切换命令 ---
  huart_->Init.BaudRate = 9600;
  HAL_UART_Init(huart_);  // dismiss return

  osDelay(200);  // 等待GY-56上电稳定

  // 0xA5 + 0x76(波特率) + 0x01(115200) + sum
  send(0x76, 0x01);

  osDelay(200);  // 等待GY-56切换

  // --- 以115200bps重新初始化 ---
  huart_->Init.BaudRate = 115200;
  HAL_UART_Init(huart_);  // dismiss return
}

// ============================================================
// 启动DMA接收
// ============================================================
void GY56::start()
{
  HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_data, sizeof(rx_data));  // dismiss return
}

// ============================================================
// 发送GY-56命令帧
// ============================================================
void GY56::send(uint8_t func, uint8_t value)
{
  tx_data[0] = 0xA5;
  tx_data[1] = func;                                             // 功能指令
  tx_data[2] = value;                                            // 指令值
  tx_data[3] = (uint8_t)(tx_data[0] + tx_data[1] + tx_data[2]);  // 校验和

  HAL_UART_Transmit(huart_, tx_data, UART_TX_BUF_LEN, 100);  // dismiss return
}

// ============================================================
// GY-56帧解析
// 在接收缓冲区中查找帧头并解析
// ============================================================
void GY56::recv(uint16_t size)
{
  for (uint16_t i = 0; i < size; ++i) {
    parse_buffer_[parse_length_++] = rx_data[i];

    // 缓冲区溢出保护
    if (parse_length_ >= sizeof(parse_buffer_)) {
      parse_length_ = 0;
      continue;
    }

    // 检查是否收到完整帧（至少9字节）
    if (parse_length_ >= FRAME_SIZE) {
      const uint16_t offset = parse_length_ - FRAME_SIZE;
      if (parse_buffer_[offset] == 0x5A && parse_buffer_[offset + 1] == 0x5A) {
        uint8_t frame[FRAME_SIZE];
        std::memcpy(frame, parse_buffer_ + offset, sizeof(frame));

        // 校验和验证
        uint8_t sum = 0;
        for (uint8_t j = 0; j < CHECKSUM_INDEX; ++j) sum += frame[j];

        if (sum == frame[CHECKSUM_INDEX]) {
          // 解析距离
          uint16_t dist_raw = ((uint16_t)frame[DIST_H_INDEX] << 8) | frame[DIST_L_INDEX];
          ir_distance_cm = (float)dist_raw;
          ir_valid = true;
        }

        // 清空缓冲区，准备接收下一帧
        parse_length_ = 0;
      }
    }
  }
}

}  // namespace sp
