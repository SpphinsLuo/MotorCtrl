// biped_hw — usb_can_adapter.hpp
// 一个 USB-CAN 适配器 = 一个串口 fd。只做 CAN 转发与握手，不使能电机、不打包 MIT。
// P0 冒烟与 P1/P2 apps 共用。
#pragma once

#include "biped_hw/serial_port.hpp"
#include "biped_hw/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>

namespace biped_hw
{

// One USB-CAN dongle = one serial fd. Framing matches all_motor.h / Damiao
// 上位机 USB2CAN and Deadline039/DM-USB2CAN firmware:
//   USB serial: B921600 (上位机 + motor_ctrl). python-can-damiao defaults to
//     2 Mbps line coding; STM32 CDC ignores that for the CAN path.
//   CAN TX: 0x55 0xAA, 30-byte frame, byte3 CMD 0x03 (forward, no TX status)
//   CAN RX: header 0xAA, CMD 0x11, end 0x55
//   Set CAN baud (on every open): 5 bytes 0x55 0x05 <index> 0xAA 0x55
//     index 0 = 1 Mbps. python-can-damiao and DM-USB2CAN both use this.
//     Firmware applies BTR; Deadline039 does not reply CMD 0x13.
//   Handshake CMD 0x02 is optional / off by default. motor_ctrl and
//     python-can-damiao never send it; a wrong 0x02 can stall some firmware.
//     --handshake on apps still sends 55 AA 1E 02 after open.
// Kernel ACM open raises DTR and can reset STM32 CDC. If DTR is wired to NRST,
// holding it asserted keeps the MCU in reset. open() clears DTR+RTS, waits
// ~2 s while draining RX, then sends the baud command up to 3 times.
struct UsbCanRxStats
{
  uint32_t polls = 0;
  uint32_t ok = 0;
  uint32_t incomplete = 0;
  uint32_t bad_cmd = 0;
  uint32_t adapter_status = 0;
  uint8_t last_bad_cmd = 0;
  uint8_t last_bad_end = 0;
  uint8_t last_status_cmd = 0;
  uint8_t last_bad_raw[16] = {};
  uint8_t last_bad_len = 0;
  uint64_t serial_bytes = 0;
  std::size_t queued_bytes = 0;
};

struct UsbCanInitStats
{
  uint8_t baud_cmd[5] = {0x55, 0x05, 0x00, 0xAA, 0x55};
  uint8_t handshake_prefix[4] = {};
  bool baud_sent = false;
  bool handshake_sent = false;
  uint32_t wait_ms = 0;
  uint8_t baud_attempts = 0;
  uint64_t usb_reply_bytes = 0;
  uint8_t last_status_cmd = 0;
  char control_tx_log[192] = {};
};

class UsbCanAdapter
{
public:
  static constexpr uint32_t kDefaultCanBitrate = 1000000;
  static constexpr std::size_t kUsbCanTxSize = 0x1e;
  // STM32 CDC re-enumerates after the DTR edge on ACM open (~1–2 s).
  static constexpr int kPostOpenWaitMs = 2000;
  static constexpr int kBaudRetryWaitMs = 300;
  static constexpr int kBaudRetries = 3;

  explicit UsbCanAdapter(std::string port,
                         speed_t baudrate = SerialPort::kUsbCanBaud,
                         int timeout_ms = SerialPort::kDefaultTimeoutMs,
                         uint32_t can_bitrate = kDefaultCanBitrate);

  bool open();
  void close();
  bool is_open() const { return serial_.is_open(); }
  int native_fd() const { return serial_.native_fd(); }
  int recv_timeout_ms() const { return serial_.timeout_ms(); }
  void set_recv_timeout(int timeout_ms) { serial_.set_timeout(timeout_ms); }
  std::size_t queued_bytes() const { return serial_.queued_bytes(); }

  const std::string &port() const { return serial_.port(); }
  const std::string &last_error() const;

  // CAN bus bitrate in bit/s. Applied automatically on open().
  bool set_can_bitrate(uint32_t bitrate);
  uint32_t can_bitrate() const { return can_bitrate_; }
  uint8_t can_baud_index() const { return can_baud_index_; }
  static bool can_bitrate_to_index(uint32_t bitrate, uint8_t *index_out);
  void fill_baud_cmd(uint8_t out[5]) const;
  const UsbCanInitStats &init_stats() const { return init_stats_; }

  // 30-byte USB2CAN TX (0x55 0xAA, CMD 0x03). Same bytes sendCan() writes.
  // Register reads use CAN ID 0x7FF; ESC 12=0x0C lives in payload byte 21, not
  // CMD/length. Byte 8 is time_interval=10 (0x0A) on every frame.
  static void pack_can_tx(uint8_t out[kUsbCanTxSize], uint32_t can_id,
                          const uint8_t *data, uint8_t len, bool extended = false);

  // Forward one CAN data frame. Default: standard data frame, 8 bytes.
  bool sendCan(uint32_t can_id, const uint8_t *data, uint8_t len = 8,
               bool extended = false);

  // True when a complete USB-CAN receive-success frame (CMD==0x11) is parsed.
  bool pollCan(CanRxFrame &out);

  // Pull already-buffered USB-CAN frames. Stops on first empty poll.
  void drain(int max_frames = 64);

  // USB2CAN CMD=2: PC <-> adapter handshake. Off during open() by default
  // (python-can-damiao / motor_ctrl never send it). --handshake still works.
  bool handshake();
  void set_handshake_on_open(bool enable) { handshake_on_open_ = enable; }
  bool handshake_on_open() const { return handshake_on_open_; }

  const UsbCanRxStats &rx_stats() const { return rx_stats_; }
  void reset_rx_stats();

private:
  void refresh_rx_byte_stats();
  bool apply_can_baud();
  bool wait_usb_activity(int timeout_ms);

  void append_ctrl_log(const char *text);

  SerialPort serial_;
  std::string last_error_;
  uint32_t can_bitrate_ = kDefaultCanBitrate;
  uint8_t can_baud_index_ = 0;
  bool handshake_on_open_ = false;
  UsbCanRxStats rx_stats_{};
  UsbCanInitStats init_stats_{};
};

}  // namespace biped_hw
