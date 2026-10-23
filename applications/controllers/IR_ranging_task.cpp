/**
 * ============================================================
 * 红外测距任务（对应任务书 4.4.2.1）
 * ============================================================
 * 1. 独立 FreeRTOS 任务负责数据获取与更新
 * 2. 无效值 / 超量程 / 通信异常统一发布为 IR_INVALID_CM(0xFFFF)
 *    - 通信异常：LaserGY56::is_valid() 自带 500 ms 超时判据
 *    - 帧校验错误 / 状态位非 0：sp::LaserGY56 内部已经丢弃
 *    - 超量程或明显异常值：这里按有效窗口再过滤一次
 * 3. 结果放在 distance 里，供折叠任务直接读取
 * 4. distance 输出做 5 点中值 + 一阶低通（约 200 ms），抑制移动时的跳变
 * ============================================================
 */

#include "can/can.hpp"
#include "io/laser/laser.hpp"

sp::LaserGY56 laser(&huart6);

// 对外发布的测距值，单位 cm；IR_INVALID_CM 表示无效/超时/超量程
volatile uint16_t distance = IR_INVALID_CM;

// 绘图用副本：无效时给 -1，避免 0xFFFF 把曲线拉爆
volatile float plot_distance_cm = -1.0f;
// 滤波前的原始读数，同样无效时给 -1
volatile float plot_distance_raw = -1.0f;

// DBus 诊断：USART3 收到的事件次数与最后一次字节数
volatile uint16_t g_dbus_events = 0;
volatile uint16_t g_dbus_last_len = 0;

namespace
{
// GY-56 出厂默认 9600，也可能被上位机改成 115200。
// 启动时两档各听一段时间，收到第一帧有效数据后锁定，不靠“发命令改波特率”
// —— 波特率不匹配时那条命令模块根本收不到。
constexpr uint32_t IR_BAUD_WINDOW_MS = 300;
constexpr uint8_t IR_BAUD_SCAN_ROUNDS = 3;
constexpr uint32_t IR_BAUDS[] = {9600U, 115200U};

// distance 输出：5 点中值 + 一阶低通。
// 这个模块比较劣质，移动时会出现单帧/双帧的离群跳变；只用低通会把尖峰
// 直接拉进输出。先用 5 点中值把离群点剔掉，再用 alpha=0.005（约 200 ms）
// 的低通磨平剩余噪声。
// 中值窗口按“新帧”推进（用 laser.rx_frames() 判断），否则 1 kHz 循环会把
// 同一个值重复塞满窗口，中值就失效了。
constexpr uint8_t DIST_MEDIAN_N = 5;
constexpr uint8_t DIST_MEDIAN_MIN = 3;  // 至少攒够 3 帧才开始输出
constexpr float DIST_LPF_ALPHA = 0.005f;

uint16_t median_of(const uint16_t * values, uint8_t n)
{
  uint16_t tmp[DIST_MEDIAN_N];
  for (uint8_t i = 0; i < n; ++i) {
    tmp[i] = values[i];
  }
  // 5 个元素，插入排序足够
  for (uint8_t i = 1; i < n; ++i) {
    const uint16_t v = tmp[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && tmp[j] > v) {
      tmp[j + 1] = tmp[j];
      --j;
    }
    tmp[j + 1] = v;
  }
  return tmp[n / 2];
}

bool wait_for_frame(uint32_t window_ms)
{
  const uint32_t frames = laser.rx_frames();
  const uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < window_ms) {
    if (laser.rx_frames() != frames) {
      return true;  // 收到至少一帧校验通过的数据
    }
    osDelay(1);
  }
  return false;
}

bool lock_baud_rate()
{
  for (uint8_t round = 0; round < IR_BAUD_SCAN_ROUNDS; ++round) {
    for (uint32_t baud : IR_BAUDS) {
      if (!laser.set_uart_baud_rate(baud)) {
        continue;
      }
      laser.start_receive();
      if (wait_for_frame(IR_BAUD_WINDOW_MS)) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace

extern "C" void IR_ranging_task(void const * argument)
{
  (void)argument;

  // 模块上电后需要一点时间才能稳定输出
  osDelay(1000);

  // 自动匹配模块当前波特率（9600 / 115200），找不到也继续跑：
  // 主循环会持续发布无效值，模块恢复后接收回调仍会更新。
  (void)lock_baud_rate();

  // 距离滤波状态：中值窗口 + 低通。
  // 中值窗口只在收到新帧时推进，低通每 1 ms 跑一次。
  uint16_t dist_hist[DIST_MEDIAN_N] = {0};
  uint8_t dist_hist_count = 0;
  uint8_t dist_hist_pos = 0;
  uint32_t dist_last_frames = laser.rx_frames();
  float dist_lpf = 0.0f;
  bool dist_lpf_ready = false;

  while (true) {
    uint16_t value = IR_INVALID_CM;

    bool raw_valid = false;
    uint16_t raw = 0;
    if (laser.is_valid()) {
      raw = laser.get_distance();
      raw_valid = (raw >= IR_MIN_VALID_CM) && (raw <= IR_MAX_VALID_CM);
    }
    plot_distance_raw = raw_valid ? static_cast<float>(raw) : -1.0f;

    if (!raw_valid) {
      // 超量程 / 超时：清空中值窗口，下一段有效数据重新起步
      dist_hist_count = 0;
      dist_hist_pos = 0;
      dist_lpf_ready = false;
      dist_last_frames = laser.rx_frames();
    }
    else {
      const uint32_t frames = laser.rx_frames();
      if (frames != dist_last_frames) {
        dist_last_frames = frames;
        dist_hist[dist_hist_pos] = raw;
        dist_hist_pos = static_cast<uint8_t>((dist_hist_pos + 1U) % DIST_MEDIAN_N);
        if (dist_hist_count < DIST_MEDIAN_N) {
          dist_hist_count++;
        }
      }

      if (dist_hist_count >= DIST_MEDIAN_MIN) {
        const uint16_t med = median_of(dist_hist, dist_hist_count);
        if (!dist_lpf_ready) {
          dist_lpf = static_cast<float>(med);
          dist_lpf_ready = true;
        }
        else {
          dist_lpf += DIST_LPF_ALPHA * (static_cast<float>(med) - dist_lpf);
        }
        value = static_cast<uint16_t>(dist_lpf + 0.5f);
      }
    }

    distance = value;
    plot_distance_cm = (value == IR_INVALID_CM) ? -1.0f : static_cast<float>(value);
    osDelay(1);
  }
}

// ---------------------------------
// 串口回调
// ---------------------------------
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  const uint32_t stamp_ms = osKernelSysTick();

  if (huart == &huart6) {
    laser.receive_complete(size);
  }
  else if (huart == &huart3) {
    g_dbus_events++;
    g_dbus_last_len = size;
    remote.update(size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart6) {
    laser.start_receive();
  }
  else if (huart == &huart3) {
    remote.request();
  }
}
