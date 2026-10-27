// biped_hw — usb_can_adapter.cpp
// UsbCanAdapter 实现：USB2CAN 0x55AA CAN 转发、0x55 0x05 设 CAN 波特率、
// 0xAA/CMD 0x11 接收帧。被 P0/P1/P2 可执行文件链接。
#include "biped_hw/usb_can_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace biped_hw
{
namespace
{

#pragma pack(push, 1)
struct UsbCanTxFrame
{
  uint8_t frame_header[2] = {0x55, 0xAA};
  uint8_t frame_len = 0x1e;
  uint8_t cmd = 0x03;  // 1: CAN forward w/ status, 2: handshake, 3: CAN no status
  uint32_t send_times = 1;
  uint32_t time_interval = 10;
  uint8_t id_type = 0;  // 0 standard, 1 extended
  uint32_t can_id = 0;
  uint8_t frame_type = 0;  // 0 data, 1 remote
  uint8_t len = 0x08;
  uint8_t id_acc = 0;
  uint8_t data_acc = 0;
  uint8_t data[8] = {};
  uint8_t crc = 0;  // unused by adapter
};

struct UsbCanRxFrame
{
  uint8_t frame_header;
  uint8_t cmd;
  uint8_t can_data_len : 6;
  uint8_t can_ide : 1;
  uint8_t can_rtr : 1;
  uint32_t can_id;
  uint8_t can_data[8];
  uint8_t frame_end;
};
#pragma pack(pop)

static_assert(sizeof(UsbCanTxFrame) == 0x1e, "USB-CAN TX frame is 30 bytes");
static_assert(sizeof(UsbCanRxFrame) == 16, "USB-CAN RX frame is 16 bytes");

constexpr uint8_t kRxHeader = 0xAA;
constexpr uint8_t kRxEnd = 0x55;
constexpr uint8_t kCmdCanForward = 0x03;
constexpr uint8_t kCmdHandshake = 0x02;
constexpr uint8_t kCmdRxOk = 0x11;
constexpr uint8_t kCmdHeartbeat = 0x00;
constexpr uint8_t kCmdTxFail = 0x02;
constexpr uint8_t kCmdTxOk = 0x12;
constexpr uint8_t kCmdBaudOk = 0x13;
constexpr uint8_t kCmdSetCanBaud = 0x05;
bool is_adapter_status_cmd(uint8_t cmd)
{
  return cmd == kCmdHeartbeat || cmd == kCmdTxFail || cmd == kCmdTxOk ||
         cmd == kCmdBaudOk;
}

}  // namespace

UsbCanAdapter::UsbCanAdapter(std::string port, speed_t baudrate, int timeout_ms,
                             uint32_t can_bitrate)
    : serial_(port, baudrate, timeout_ms), port_(std::move(port)), timeout_ms_(timeout_ms)
{
  set_can_bitrate(can_bitrate);
}

const std::string &UsbCanAdapter::last_error() const
{
  return last_error_.empty() && !is_socketcan() ? serial_.last_error() : last_error_;
}

bool UsbCanAdapter::is_socketcan() const
{
  return port_.rfind("socketcan:", 0) == 0;
}

bool UsbCanAdapter::is_open() const
{
  return is_socketcan() ? socket_fd_ >= 0 : serial_.is_open();
}

int UsbCanAdapter::native_fd() const
{
  return is_socketcan() ? socket_fd_ : serial_.native_fd();
}

int UsbCanAdapter::recv_timeout_ms() const
{
  return is_socketcan() ? timeout_ms_ : serial_.timeout_ms();
}

void UsbCanAdapter::set_recv_timeout(int timeout_ms)
{
  timeout_ms_ = timeout_ms < 0 ? 0 : timeout_ms;
  if (!is_socketcan())
  {
    serial_.set_timeout(timeout_ms_);
  }
}

std::size_t UsbCanAdapter::queued_bytes() const
{
  return is_socketcan() ? 0 : serial_.queued_bytes();
}

bool UsbCanAdapter::open_socketcan()
{
  const std::string ifname = port_.substr(std::strlen("socketcan:"));
  if (ifname.empty() || ifname.size() >= IFNAMSIZ)
  {
    last_error_ = "invalid SocketCAN interface (expected socketcan:canX)";
    return false;
  }
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0)
  {
    last_error_ = std::string("SocketCAN socket: ") + std::strerror(errno);
    return false;
  }
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    last_error_ = std::string("SocketCAN nonblocking: ") + std::strerror(errno);
    close_socketcan();
    return false;
  }
  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    last_error_ = std::string("SocketCAN interface ") + ifname + ": " + std::strerror(errno);
    close_socketcan();
    return false;
  }
  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    last_error_ = std::string("SocketCAN bind ") + ifname + ": " + std::strerror(errno);
    close_socketcan();
    return false;
  }
  append_ctrl_log("backend=socketcan");
  return true;
}

void UsbCanAdapter::close_socketcan()
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool UsbCanAdapter::can_bitrate_to_index(uint32_t bitrate, uint8_t *index_out)
{
  uint8_t idx = 0;
  switch (bitrate)
  {
  case 1000000:
    idx = 0;
    break;
  case 800000:
    idx = 1;
    break;
  case 666000:
  case 666666:
    idx = 2;
    break;
  case 500000:
    idx = 3;
    break;
  case 400000:
    idx = 4;
    break;
  case 250000:
    idx = 5;
    break;
  case 200000:
    idx = 6;
    break;
  case 125000:
    idx = 7;
    break;
  case 100000:
    idx = 8;
    break;
  case 80000:
    idx = 9;
    break;
  case 50000:
    idx = 10;
    break;
  case 40000:
    idx = 11;
    break;
  case 20000:
    idx = 12;
    break;
  case 10000:
    idx = 13;
    break;
  case 5000:
    idx = 14;
    break;
  default:
    return false;
  }
  if (index_out != nullptr)
  {
    *index_out = idx;
  }
  return true;
}

bool UsbCanAdapter::set_can_bitrate(uint32_t bitrate)
{
  uint8_t idx = 0;
  if (!can_bitrate_to_index(bitrate, &idx))
  {
    last_error_ = "unsupported CAN bitrate";
    return false;
  }
  can_bitrate_ = bitrate;
  can_baud_index_ = idx;
  last_error_.clear();
  return true;
}

void UsbCanAdapter::fill_baud_cmd(uint8_t out[5]) const
{
  out[0] = 0x55;
  out[1] = kCmdSetCanBaud;
  out[2] = can_baud_index_;
  out[3] = 0xAA;
  out[4] = 0x55;
}

bool UsbCanAdapter::apply_can_baud()
{
  if (is_socketcan())
  {
    return true;
  }
  if (!serial_.is_open())
  {
    last_error_ = "port not open";
    return false;
  }
  uint8_t cmd[5] = {};
  fill_baud_cmd(cmd);
  const ssize_t n = serial_.send(cmd, sizeof(cmd));
  serial_.drain_output();
  if (n != static_cast<ssize_t>(sizeof(cmd)))
  {
    last_error_ = "USB2CAN set-CAN-baud write failed";
    return false;
  }
  init_stats_.baud_sent = true;
  std::memcpy(init_stats_.baud_cmd, cmd, sizeof(cmd));
  return true;
}

bool UsbCanAdapter::wait_usb_activity(int timeout_ms)
{
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  const uint64_t start = serial_.bytes_read();
  CanRxFrame unused;
  while (std::chrono::steady_clock::now() < deadline)
  {
    pollCan(unused);
    if (serial_.bytes_read() > start)
    {
      return true;
    }
  }
  return serial_.bytes_read() > start;
}

void UsbCanAdapter::refresh_rx_byte_stats()
{
  if (is_socketcan())
  {
    return;
  }
  rx_stats_.serial_bytes = serial_.bytes_read();
  rx_stats_.queued_bytes = serial_.queued_bytes();
}

void UsbCanAdapter::reset_rx_stats()
{
  rx_stats_ = {};
  if (!is_socketcan())
  {
    serial_.reset_stats();
  }
}

void UsbCanAdapter::append_ctrl_log(const char *text)
{
  if (text == nullptr || text[0] == '\0')
  {
    return;
  }
  const std::size_t used = std::strlen(init_stats_.control_tx_log);
  const std::size_t cap = sizeof(init_stats_.control_tx_log);
  if (used + 1 >= cap)
  {
    return;
  }
  const char *sep = used == 0 ? "" : " ";
  std::snprintf(init_stats_.control_tx_log + used, cap - used, "%s%s", sep, text);
}

bool UsbCanAdapter::open()
{
  last_error_.clear();
  init_stats_ = {};
  if (is_socketcan())
  {
    return open_socketcan();
  }
  fill_baud_cmd(init_stats_.baud_cmd);
  init_stats_.wait_ms = static_cast<uint32_t>(kPostOpenWaitMs);
  if (!serial_.open())
  {
    return false;
  }

  // Kernel ACM open already raised DTR. If DTR is wired to NRST (inverted),
  // leaving it asserted holds reset — drop DTR+RTS and do not raise them again.
  serial_.set_dtr_rts(false);
  append_ctrl_log("dtr=0");

  char wait_note[48];
  std::snprintf(wait_note, sizeof(wait_note), "wait_ms=%d", kPostOpenWaitMs);
  append_ctrl_log(wait_note);

  serial_.reset_stats();
  reset_rx_stats();
  wait_usb_activity(kPostOpenWaitMs);

  for (int attempt = 0; attempt < kBaudRetries; ++attempt)
  {
    if (!apply_can_baud())
    {
      serial_.close();
      return false;
    }
    init_stats_.baud_attempts = static_cast<uint8_t>(attempt + 1);
    char baud_note[48];
    std::snprintf(baud_note, sizeof(baud_note), "baud#%d=55 05 %02X AA 55",
                  attempt + 1, static_cast<unsigned>(can_baud_index_));
    append_ctrl_log(baud_note);
    if (wait_usb_activity(kBaudRetryWaitMs))
    {
      break;
    }
  }

  if (handshake_on_open_)
  {
    if (!handshake())
    {
      last_error_ = "USB2CAN handshake write failed";
      serial_.close();
      return false;
    }
    append_ctrl_log("handshake=55 AA 1E 02");
    wait_usb_activity(200);
  }
  else
  {
    append_ctrl_log("handshake=off");
  }

  init_stats_.usb_reply_bytes = serial_.bytes_read();
  init_stats_.last_status_cmd = rx_stats_.last_status_cmd;
  drain();
  reset_rx_stats();
  if (init_stats_.usb_reply_bytes == 0)
  {
    last_error_ = handshake_on_open_
                      ? "USB2CAN silent after 55 05 <idx> AA 55 + handshake (no USB RX)"
                      : "USB2CAN silent after 55 05 <idx> AA 55 (no USB RX)";
  }
  return true;
}

void UsbCanAdapter::close()
{
  if (is_socketcan())
  {
    close_socketcan();
  }
  else
  {
    serial_.close();
  }
}

void UsbCanAdapter::pack_can_tx(uint8_t out[kUsbCanTxSize], uint32_t can_id,
                                const uint8_t *data, uint8_t len, bool extended)
{
  UsbCanTxFrame tx{};
  tx.cmd = kCmdCanForward;
  tx.can_id = can_id;
  tx.id_type = extended ? 1 : 0;
  tx.len = len > 8 ? 8 : len;
  std::memset(tx.data, 0, sizeof(tx.data));
  if (data != nullptr && tx.len > 0)
  {
    std::copy(data, data + tx.len, tx.data);
  }
  std::memcpy(out, &tx, kUsbCanTxSize);
}

bool UsbCanAdapter::sendCan(uint32_t can_id, const uint8_t *data, uint8_t len,
                            bool extended)
{
  if (!is_open())
  {
    return false;
  }
  if (len > 8)
  {
    return false;
  }

  if (is_socketcan())
  {
    return send_socketcan(can_id, data, len, extended);
  }
  uint8_t raw[kUsbCanTxSize] = {};
  pack_can_tx(raw, can_id, data, len, extended);
  const ssize_t n = serial_.send(raw, sizeof(raw));
  return n == static_cast<ssize_t>(sizeof(raw));
}

bool UsbCanAdapter::send_socketcan(uint32_t can_id, const uint8_t *data, uint8_t len,
                                   bool extended)
{
  struct can_frame frame {};
  frame.can_id = can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  if (extended)
  {
    frame.can_id |= CAN_EFF_FLAG;
  }
  frame.len = len;
  if (data != nullptr && len > 0)
  {
    std::memcpy(frame.data, data, len);
  }
  const ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

bool UsbCanAdapter::pollCan(CanRxFrame &out)
{
  if (is_socketcan())
  {
    return poll_socketcan(out);
  }
  if (!serial_.is_open())
  {
    return false;
  }

  ++rx_stats_.polls;
  UsbCanRxFrame rx{};
  serial_.recv(reinterpret_cast<uint8_t *>(&rx), kRxHeader, sizeof(rx));
  refresh_rx_byte_stats();
  if (rx.frame_header != kRxHeader)
  {
    ++rx_stats_.incomplete;
    return false;
  }
  if (rx.frame_end == kRxEnd && is_adapter_status_cmd(rx.cmd))
  {
    ++rx_stats_.adapter_status;
    rx_stats_.last_status_cmd = rx.cmd;
    return false;
  }
  if (rx.cmd != kCmdRxOk || rx.frame_end != kRxEnd)
  {
    ++rx_stats_.bad_cmd;
    rx_stats_.last_bad_cmd = rx.cmd;
    rx_stats_.last_bad_end = rx.frame_end;
    std::memcpy(rx_stats_.last_bad_raw, &rx, sizeof(rx));
    rx_stats_.last_bad_len = static_cast<uint8_t>(sizeof(rx));
    return false;
  }

  out.can_id = rx.can_id;
  out.dlc = rx.can_data_len;
  if (out.dlc > 8)
  {
    out.dlc = 8;
  }
  std::memcpy(out.data, rx.can_data, 8);
  out.extended = rx.can_ide != 0;
  out.remote = rx.can_rtr != 0;
  ++rx_stats_.ok;
  return true;
}

bool UsbCanAdapter::poll_socketcan(CanRxFrame &out)
{
  if (socket_fd_ < 0)
  {
    return false;
  }
  struct can_frame frame {};
  const ssize_t n = ::recv(socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (n < 0)
  {
    return false;
  }
  if (n != static_cast<ssize_t>(sizeof(frame)) || frame.len > 8)
  {
    ++rx_stats_.incomplete;
    return false;
  }
  out.can_id = frame.can_id & (frame.can_id & CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
  out.extended = (frame.can_id & CAN_EFF_FLAG) != 0;
  out.remote = (frame.can_id & CAN_RTR_FLAG) != 0;
  out.dlc = frame.len;
  std::memcpy(out.data, frame.data, frame.len);
  ++rx_stats_.ok;
  return true;
}

void UsbCanAdapter::drain(int max_frames)
{
  CanRxFrame unused;
  int n = 0;
  while (n < max_frames && pollCan(unused))
  {
    ++n;
  }
}

bool UsbCanAdapter::handshake()
{
  if (is_socketcan())
  {
    return is_open();
  }
  if (!serial_.is_open())
  {
    return false;
  }
  UsbCanTxFrame tx{};
  tx.cmd = kCmdHandshake;
  const ssize_t n = serial_.send(reinterpret_cast<const uint8_t *>(&tx), sizeof(tx));
  serial_.drain_output();
  if (n == static_cast<ssize_t>(sizeof(tx)))
  {
    init_stats_.handshake_sent = true;
    init_stats_.handshake_prefix[0] = tx.frame_header[0];
    init_stats_.handshake_prefix[1] = tx.frame_header[1];
    init_stats_.handshake_prefix[2] = tx.frame_len;
    init_stats_.handshake_prefix[3] = tx.cmd;
    return true;
  }
  return false;
}

}  // namespace biped_hw
