// biped_hw — serial_port.hpp
// USB-CAN 串口封装：921600 baud、select 超时、打开失败返回 false（不 exit）。
// Open uses cfmakeraw so writes pass 0x00–0xFF (CAN ID 10=LF, 12=FF, 13=CR).
// UsbCanAdapter 使用；实现见 src/serial_port.cpp。
#pragma once

#include <array>
#include <cstdint>
#include <queue>
#include <string>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace biped_hw
{

// Salvaged from motor_ctrl/SerialPort.h. Open failure returns false; never
// calls exit(). Baud for USB-CAN is B921600. Recv uses select() timeout.
class SerialPort
{
public:
  static constexpr speed_t kUsbCanBaud = B921600;
  static constexpr int kDefaultTimeoutMs = 2;

  SerialPort(std::string port, speed_t baudrate = kUsbCanBaud,
             int timeout_ms = kDefaultTimeoutMs, unsigned baud_hz = 0);
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;
  SerialPort(SerialPort &&) = delete;
  SerialPort &operator=(SerialPort &&) = delete;

  bool open();
  void close();
  bool is_open() const { return fd_ >= 0; }
  int native_fd() const { return fd_; }
  int timeout_ms() const { return timeout_ms_; }

  const std::string &port() const { return port_; }
  const std::string &last_error() const { return last_error_; }

  ssize_t send(const uint8_t *data, size_t len);
  ssize_t recv(uint8_t *data, size_t len);
  // Block until `len` bytes are read or `timeout_ms` elapses. Does not return
  // after the first byte — USB-UART often delivers one character per URB.
  // Returns bytes obtained (0 on timeout with nothing, -1 on error).
  ssize_t recv_n(uint8_t *data, size_t len, int timeout_ms);
  // Drain into a queue, then copy one frame that starts with `head`.
  void recv(uint8_t *data, uint8_t head, ssize_t len);

  // Wait until the kernel has shifted out queued TX (baud/handshake bytes).
  bool drain_output();
  // Drop unread RX (leftover IMU stream, previous poll). TX untouched.
  bool flush_input();
  // Set or clear DTR+RTS. Kernel ACM open already raises DTR (STM32 CDC
  // often resets on that edge). Do not leave DTR asserted if that holds NRST.
  // ~HUPCL still avoids a DTR drop on close.
  bool set_dtr_rts(bool asserted);
  // RTS only. USB-RS485 boards that wire RTS as DE/RE need it pulsed.
  // Boards that auto-DE from UART TX often require RTS held *off* (clear
  // TIOCM_RTS); leaving the kernel open-time RTS assertion (DTR+RTS) stuck
  // on keeps DE asserted so the encoder never gets the bus. Does not touch DTR.
  bool set_rts(bool asserted);

  void set_timeout(int timeout_ms);

  // Numeric baud requested at construction (0 if unknown).
  unsigned requested_baud_hz() const { return requested_baud_hz_; }
  // Linux TCGETS2 c_ospeed after configure, 0 if the ioctl failed.
  unsigned linux_baud_hz() const { return linux_baud_hz_; }

  uint64_t bytes_read() const { return bytes_read_; }
  std::size_t queued_bytes() const { return recv_queue_.size(); }
  void reset_stats() { bytes_read_ = 0; }

private:
  bool configure();
  bool apply_linux_baud(unsigned baud_hz);
  unsigned query_linux_baud() const;

  std::string port_;
  speed_t baudrate_;
  unsigned requested_baud_hz_ = 0;
  unsigned linux_baud_hz_ = 0;
  int timeout_ms_;
  int fd_ = -1;
  std::string last_error_;
  uint64_t bytes_read_ = 0;

  fd_set rset_{};
  std::queue<uint8_t> recv_queue_;
  std::array<uint8_t, 1024> recv_buf_{};
};

// Kernel tty driver name (e.g. pl2303, cp210x) from /sys. Empty if unknown.
std::string tty_driver_name(const std::string &port);

}  // namespace biped_hw
