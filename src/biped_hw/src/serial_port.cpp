// biped_hw — serial_port.cpp
// SerialPort 实现：termios 配置、open/close、send/recv 与按帧头组包。
// 被 UsbCanAdapter 链接进 P0/P1/P2 可执行文件。
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#include "biped_hw/serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>

namespace biped_hw
{
namespace
{

// Kernel termios2 (numeric baud). Do not include linux/termios.h with glibc
// termios.h — the structs collide. Layout matches asm-generic/termbits.h.
struct LinuxTermios2
{
  tcflag_t c_iflag;
  tcflag_t c_oflag;
  tcflag_t c_cflag;
  tcflag_t c_lflag;
  cc_t c_line;
  cc_t c_cc[19];
  speed_t c_ispeed;
  speed_t c_ospeed;
};

#ifndef BOTHER
#define BOTHER 0010000
#endif

// sys/ioctl.h already defines TCGETS2 in terms of incomplete struct termios2.
// Use our layout with the same ioctl numbers.
constexpr unsigned long kTcGets2 = _IOR('T', 0x2A, LinuxTermios2);
constexpr unsigned long kTcSets2 = _IOW('T', 0x2B, LinuxTermios2);

unsigned speed_to_hz(speed_t s)
{
  switch (s)
  {
  case B115200:
    return 115200;
  case B230400:
    return 230400;
  case B460800:
    return 460800;
  case B921600:
    return 921600;
#ifdef B1000000
  case B1000000:
    return 1000000;
#endif
#ifdef B2000000
  case B2000000:
    return 2000000;
#endif
#ifdef B2500000
  case B2500000:
    return 2500000;
#endif
#ifdef B3000000
  case B3000000:
    return 3000000;
#endif
  default:
    return 0;
  }
}

}  // namespace

SerialPort::SerialPort(std::string port, speed_t baudrate, int timeout_ms,
                       unsigned baud_hz)
    : port_(std::move(port)), baudrate_(baudrate), requested_baud_hz_(baud_hz),
      timeout_ms_(timeout_ms)
{
  set_timeout(timeout_ms);
  if (requested_baud_hz_ == 0)
  {
    requested_baud_hz_ = speed_to_hz(baudrate_);
  }
}

SerialPort::~SerialPort()
{
  close();
}

void SerialPort::set_timeout(int timeout_ms)
{
  timeout_ms_ = timeout_ms < 0 ? 0 : timeout_ms;
}

bool SerialPort::configure()
{
  struct termios option;
  std::memset(&option, 0, sizeof(option));
  if (tcgetattr(fd_, &option) < 0)
  {
    last_error_ = std::strerror(errno);
    return false;
  }

  // USB2CAN frames are binary. The CAN-ID field is a little-endian uint32, so
  // decimal ID 10/12/13 put 0x0A (LF) / 0x0C (FF) / 0x0D (CR) on the wire.
  // N_TTY OPOST/ONLCR rewrites NL→CRLF on write; ICRNL/INLCR eat or remap RX.
  // IDs 1–9 never hit those bytes, which looked like "two-digit IDs fail".
  // Zeroing c_*flag is not enough: cfmakeraw is the N_TTY raw contract, then
  // re-clear the map bits it leaves and restore CREAD|CLOCAL / CS8 / ~HUPCL.
  cfmakeraw(&option);
  option.c_iflag &= static_cast<tcflag_t>(
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON));
#ifdef IXOFF
  option.c_iflag &= static_cast<tcflag_t>(~IXOFF);
#endif
#ifdef IXANY
  option.c_iflag &= static_cast<tcflag_t>(~IXANY);
#endif
  option.c_oflag &= static_cast<tcflag_t>(~(OPOST | ONLCR | OCRNL));
#ifdef ONOCR
  option.c_oflag &= static_cast<tcflag_t>(~ONOCR);
#endif
#ifdef ONLRET
  option.c_oflag &= static_cast<tcflag_t>(~ONLRET);
#endif
  option.c_lflag &= static_cast<tcflag_t>(
      ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | ISIG | IEXTEN));

  cfsetispeed(&option, baudrate_);
  cfsetospeed(&option, baudrate_);

  option.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB | HUPCL));
#ifdef CRTSCTS
  option.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  // 8N1. MBS BUS-D datasheet is 8 bit / 奇偶校验=无 / 1 stop — odd parity is
  // request bit b7, not a 9th MARK/SPACE/PARENB bit. Keep PARENB off.
  // CREAD must be set or the UART will TX but never deliver RX bytes.
  // CLOCAL ignores modem CD; clearing HUPCL avoids DTR drop on close (some
  // USB2CAN dongles reset CAN baud when DTR falls).
  option.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);

  // Non-blocking read; timeout is select() in recv(). cfmakeraw sets VMIN=1.
  option.c_cc[VTIME] = 0;
  option.c_cc[VMIN] = 0;
  // Do not copy vendor SerialPort.h's `c_lflag |= CBAUDEX`: that bit is FLUSHO.

  if (tcflush(fd_, TCIOFLUSH) < 0)
  {
    last_error_ = std::strerror(errno);
    return false;
  }
  if (tcsetattr(fd_, TCSANOW, &option) < 0)
  {
    last_error_ = std::strerror(errno);
    return false;
  }

  struct termios got;
  std::memset(&got, 0, sizeof(got));
  if (tcgetattr(fd_, &got) == 0)
  {
    if ((got.c_oflag & OPOST) != 0 ||
        (got.c_iflag & static_cast<tcflag_t>(ICRNL | INLCR | IGNCR | IXON)) != 0 ||
        (got.c_lflag & static_cast<tcflag_t>(ICANON | ECHO)) != 0)
    {
      last_error_ = "termios not raw after tcsetattr (OPOST/ICRNL/ICANON stuck)";
      return false;
    }
  }

  // B2500000 is a Linux B* flag, but CH340/CP2102/FT232 often ignore it or
  // silently pick another rate. termios2 BOTHER asks for the numeric Hz;
  // FTDI may still need a custom divisor.
  if (requested_baud_hz_ == 2500000u || requested_baud_hz_ == 2000000u ||
      requested_baud_hz_ == 3000000u)
  {
    apply_linux_baud(requested_baud_hz_);
  }
  linux_baud_hz_ = query_linux_baud();
  return true;
}

unsigned SerialPort::query_linux_baud() const
{
  if (fd_ < 0)
  {
    return 0;
  }
  LinuxTermios2 t2{};
  if (ioctl(fd_, kTcGets2, &t2) == 0 && t2.c_ospeed != 0)
  {
    return static_cast<unsigned>(t2.c_ospeed);
  }
  return speed_to_hz(baudrate_);
}

bool SerialPort::apply_linux_baud(unsigned baud_hz)
{
  if (fd_ < 0 || baud_hz == 0)
  {
    return false;
  }

  LinuxTermios2 t2{};
  if (ioctl(fd_, kTcGets2, &t2) == 0)
  {
    t2.c_cflag &= static_cast<tcflag_t>(~CBAUD);
    t2.c_cflag |= static_cast<tcflag_t>(BOTHER);
    t2.c_ispeed = static_cast<speed_t>(baud_hz);
    t2.c_ospeed = static_cast<speed_t>(baud_hz);
    if (ioctl(fd_, kTcSets2, &t2) == 0)
    {
      LinuxTermios2 got{};
      if (ioctl(fd_, kTcGets2, &got) == 0 && got.c_ospeed == baud_hz)
      {
        return true;
      }
    }
  }

  // FTDI legacy: B38400 + custom_divisor. May need CAP_SYS_ADMIN.
  struct serial_struct ss;
  std::memset(&ss, 0, sizeof(ss));
  if (ioctl(fd_, TIOCGSERIAL, &ss) < 0 || ss.baud_base <= 0)
  {
    return false;
  }
  ss.custom_divisor = static_cast<int>((ss.baud_base + static_cast<int>(baud_hz) / 2) /
                                       static_cast<int>(baud_hz));
  if (ss.custom_divisor < 1)
  {
    ss.custom_divisor = 1;
  }
  ss.flags &= ~ASYNC_SPD_MASK;
  ss.flags |= ASYNC_SPD_CUST;
  if (ioctl(fd_, TIOCSSERIAL, &ss) < 0)
  {
    return false;
  }

  struct termios option;
  std::memset(&option, 0, sizeof(option));
  if (tcgetattr(fd_, &option) < 0)
  {
    return false;
  }
  cfsetispeed(&option, B38400);
  cfsetospeed(&option, B38400);
  return tcsetattr(fd_, TCSANOW, &option) == 0;
}

bool SerialPort::open()
{
  if (fd_ >= 0)
  {
    return true;
  }

  // O_NONBLOCK: CDC ACM open can block on DCD before CLOCAL is set.
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    last_error_ = std::strerror(errno);
    fd_ = -1;
    return false;
  }

  const int flags = fcntl(fd_, F_GETFL, 0);
  if (flags >= 0)
  {
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
  }

  if (!configure())
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  last_error_.clear();
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::drain_output()
{
  if (fd_ < 0)
  {
    return false;
  }
  return tcdrain(fd_) == 0;
}

bool SerialPort::flush_input()
{
  if (fd_ < 0)
  {
    return false;
  }
  return tcflush(fd_, TCIFLUSH) == 0;
}

bool SerialPort::set_dtr_rts(bool asserted)
{
  if (fd_ < 0)
  {
    return false;
  }
  int bits = TIOCM_DTR | TIOCM_RTS;
  return ioctl(fd_, asserted ? TIOCMBIS : TIOCMBIC, &bits) >= 0;
}

bool SerialPort::set_rts(bool asserted)
{
  if (fd_ < 0)
  {
    return false;
  }
  // TIOCMBIC/BIS on TIOCM_RTS only — DTR stays at whatever open() left it.
  int bits = TIOCM_RTS;
  return ioctl(fd_, asserted ? TIOCMBIS : TIOCMBIC, &bits) >= 0;
}

ssize_t SerialPort::send(const uint8_t *data, size_t len)
{
  if (fd_ < 0 || data == nullptr)
  {
    return -1;
  }
  return ::write(fd_, data, len);
}

ssize_t SerialPort::recv(uint8_t *data, size_t len)
{
  if (fd_ < 0 || data == nullptr || len == 0)
  {
    return -1;
  }

  FD_ZERO(&rset_);
  FD_SET(fd_, &rset_);

  timeval timeout{};
  timeout.tv_sec = timeout_ms_ / 1000;
  timeout.tv_usec = (timeout_ms_ % 1000) * 1000;

  ssize_t recv_len = 0;
  switch (select(fd_ + 1, &rset_, nullptr, nullptr, &timeout))
  {
  case -1:
  case 0:
    break;
  default:
    recv_len = ::read(fd_, data, len);
    break;
  }
  if (recv_len > 0)
  {
    bytes_read_ += static_cast<uint64_t>(recv_len);
  }
  return recv_len;
}

ssize_t SerialPort::recv_n(uint8_t *data, size_t len, int timeout_ms)
{
  if (fd_ < 0 || data == nullptr || len == 0)
  {
    return -1;
  }

  const int wait_ms = timeout_ms < 0 ? 0 : timeout_ms;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(wait_ms);
  size_t got = 0;

  while (got < len)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline)
    {
      break;
    }
    const auto left =
        std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
    if (left.count() <= 0)
    {
      break;
    }

    FD_ZERO(&rset_);
    FD_SET(fd_, &rset_);
    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(left.count() / 1000000);
    timeout.tv_usec = static_cast<suseconds_t>(left.count() % 1000000);

    const int sel = ::select(fd_ + 1, &rset_, nullptr, nullptr, &timeout);
    if (sel < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      return got > 0 ? static_cast<ssize_t>(got) : -1;
    }
    if (sel == 0)
    {
      break;
    }

    const ssize_t n = ::read(fd_, data + got, len - got);
    if (n < 0)
    {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
      {
        continue;
      }
      return got > 0 ? static_cast<ssize_t>(got) : -1;
    }
    if (n == 0)
    {
      break;
    }
    got += static_cast<size_t>(n);
    bytes_read_ += static_cast<uint64_t>(n);
  }

  return static_cast<ssize_t>(got);
}

void SerialPort::recv(uint8_t *data, uint8_t head, ssize_t len)
{
  if (data == nullptr || len <= 0)
  {
    return;
  }

  const ssize_t n = this->recv(recv_buf_.data(), static_cast<size_t>(len));
  for (ssize_t i = 0; i < n; ++i)
  {
    recv_queue_.push(recv_buf_[static_cast<size_t>(i)]);
  }

  while (recv_queue_.size() >= static_cast<size_t>(len))
  {
    if (recv_queue_.front() != head)
    {
      recv_queue_.pop();
      continue;
    }
    break;
  }

  if (recv_queue_.size() < static_cast<size_t>(len))
  {
    return;
  }

  for (ssize_t i = 0; i < len; ++i)
  {
    data[i] = recv_queue_.front();
    recv_queue_.pop();
  }
}

}  // namespace biped_hw
