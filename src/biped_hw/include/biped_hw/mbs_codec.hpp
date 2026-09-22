// biped_hw — mbs_codec.hpp
// 金钢 MBS 磁编码器 UART/RS485 编解码（数据手册 v2.7）。
//
// 型号后缀决定协议，不是同一套帧：
//   R  = RS485 无地址：主机发 1 字节 0x64/0x31/0x74，回 N 字节 + CRC-8 poly 0x97。
//   D  = BUS 高速总线：电气仍是二线 485，UART 8N1 **固定 2.5 Mbps**。主机发 1 字节
//        [b7 奇校验 | b6:5 操作 | b4:0 地址]，出厂地址 0x1F。回 B0=请求回显 +
//        数据 + CRC-8 (x^8+1) = 各字节异或。获取信息 op=00；op=11 无应答。
//   附加 A=115200 仅接口 R/A 可选；D 没有 115200 档。SKU 末尾 A-M 是温度/无附加。
// SSI(S) / BISS-C / T485 / PERIOD 不在这里处理。
//
// USB-RS485 常把 TX 回显进 RX；半双工无自动 DE 的 pl2303 需 RTS 作 DE。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>

namespace biped_hw
{

inline constexpr uint8_t kMbsCmdSetZero = 0x30;
inline constexpr uint8_t kMbsCmdGetPos = 0x31;
inline constexpr uint8_t kMbsCmdGetPosStatus = 0x64;
inline constexpr uint8_t kMbsCmdGetPosTemp = 0x74;

inline constexpr unsigned kMbsBaudDefault = 2500000;
inline constexpr unsigned kMbsBaud115200 = 115200;
inline constexpr unsigned kMbsBusBaudDefault = 2500000;
inline constexpr uint8_t kMbsBusAddrDefault = 0x1F;  // datasheet: 出厂默认 0x1F
inline constexpr uint8_t kMbsBusAddrMask = 0x1F;
// Empty on purpose: do not default to /dev/ttyUSB0. N100 IMU (cp210x) and
// MBS USB-RS485 (pl2303) both enumerate as ttyUSB* and swap with plug order.
// mbs_encoder_recv resolves /dev/serial/by-id from config/mbs_encoder.yaml.
inline constexpr const char *kMbsDefaultPort = "";
inline constexpr const char *kMbsSkuUser = "MBS-15-45-A-17-D-N-A-M";

inline constexpr double kMbsPi = 3.14159265358979323846;

// CRC-8 table, poly x^8+x^7+x^4+x^2+x+1. Copied from datasheet appendix.
inline constexpr uint8_t kMbsCrc8Table[256] = {
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73,
    0xB8, 0x2F, 0x01, 0x96, 0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71,
    0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C, 0xE3, 0x74, 0x5A, 0xCD,
    0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A,
    0xE1, 0x76, 0x58, 0xCF, 0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A,
    0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7, 0xEB, 0x7C, 0x52, 0xC5,
    0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1,
    0x0A, 0x9D, 0xB3, 0x24, 0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3,
    0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E, 0xA2, 0x35, 0x1B, 0x8C,
    0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B,
    0xA0, 0x37, 0x19, 0x8E, 0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A,
    0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7, 0xFB, 0x6C, 0x42, 0xD5,
    0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80,
    0x4B, 0xDC, 0xF2, 0x65, 0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82,
    0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF, 0x10, 0x87, 0xA9, 0x3E,
    0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9,
    0x12, 0x85, 0xAB, 0x3C,
};

// Datasheet calcCRC: temp = first byte, then temp = next ^ table[temp], return
// table[temp]. Equivalent to table-driven CRC-8, init 0, poly 0x97.
inline uint8_t mbs_crc8(const uint8_t *data, std::size_t length)
{
  if (data == nullptr || length == 0)
  {
    return 0;
  }
  uint8_t temp = data[0];
  for (std::size_t i = 1; i < length; ++i)
  {
    temp = static_cast<uint8_t>(data[i] ^ kMbsCrc8Table[temp]);
  }
  return kMbsCrc8Table[temp];
}

inline bool mbs_crc8_ok(const uint8_t *frame, std::size_t length)
{
  if (frame == nullptr || length < 2)
  {
    return false;
  }
  const uint8_t got = frame[length - 1];
  const uint8_t want = mbs_crc8(frame, length - 1);
  return got == want;
}

// USB-RS485 local echo: the command byte (0x64/0x31/0x74) comes back first.
// Strip a lone echo, or echo+full frame. Do not strip a CRC-valid reply
// (A2 can legally be 0x64) or a short partial payload.
inline std::size_t mbs_strip_leading_echo(uint8_t *buf, std::size_t n, uint8_t cmd,
                                          std::size_t reply_n)
{
  if (buf == nullptr || n == 0 || buf[0] != cmd)
  {
    return n;
  }
  if (n >= reply_n && mbs_crc8_ok(buf, reply_n))
  {
    return n;
  }
  if (n == 1 || n >= reply_n + 1)
  {
    std::memmove(buf, buf + 1, n - 1);
    return n - 1;
  }
  return n;
}

enum class MbsCmd : uint8_t
{
  GetPos = kMbsCmdGetPos,
  GetPosStatus = kMbsCmdGetPosStatus,
  GetPosTemp = kMbsCmdGetPosTemp,
};

struct MbsLayout
{
  uint8_t single_bits = 17;  // 16 or 17
  bool multi = false;
  MbsCmd cmd = MbsCmd::GetPosStatus;
};

inline bool mbs_layout_valid(const MbsLayout &lay)
{
  return lay.single_bits == 16 || lay.single_bits == 17;
}

inline std::size_t mbs_angle_bytes(const MbsLayout &lay)
{
  return lay.single_bits > 16 ? 3u : 2u;
}

// Reply length N from datasheet RS485 table (includes CRC, no header).
inline std::size_t mbs_reply_size(const MbsLayout &lay)
{
  std::size_t n = 0;
  if (lay.multi)
  {
    n += 2;
  }
  n += mbs_angle_bytes(lay);
  if (lay.cmd == MbsCmd::GetPosStatus)
  {
    n += 1;  // S
  }
  else if (lay.cmd == MbsCmd::GetPosTemp)
  {
    n += 2;  // T1 T0, int16 / 10 °C
  }
  n += 1;  // CRC
  return n;
}

// Status byte S (0x64). Same bit map as SSI/PERIOD. Error/warn are b7/b6.
struct MbsStatus
{
  uint8_t raw = 0;
  bool error = false;          // b7: data invalid when set
  bool warning = false;        // b6: still valid, near a limit
  bool batt_or_overrot = false;  // b5: BM battery open / FM >±90° while off
  bool batt_low = false;       // b4
  bool field_strong = false;   // b3
  bool field_weak = false;     // b2
  bool temp_out = false;       // b1
};

inline MbsStatus mbs_decode_status(uint8_t s)
{
  MbsStatus st;
  st.raw = s;
  st.error = (s & 0x80u) != 0;
  st.warning = (s & 0x40u) != 0;
  st.batt_or_overrot = (s & 0x20u) != 0;
  st.batt_low = (s & 0x10u) != 0;
  st.field_strong = (s & 0x08u) != 0;
  st.field_weak = (s & 0x04u) != 0;
  st.temp_out = (s & 0x02u) != 0;
  return st;
}

struct MbsSample
{
  uint16_t multi_raw = 0;
  int16_t multi_turns = 0;
  uint32_t angle_raw = 0;
  double angle_deg = 0.0;        // single-turn 0..360
  double angle_rad = 0.0;        // single-turn 0..2π
  double total_deg = 0.0;        // multi*360 + single
  double total_rad = 0.0;
  bool has_status = false;
  MbsStatus status{};
  bool has_temp = false;
  float temp_c = 0.f;
  uint8_t crc = 0;
  std::array<uint8_t, 8> raw{};
  uint8_t raw_len = 0;
};

inline bool mbs_unpack(const uint8_t *frame, std::size_t length, const MbsLayout &lay,
                       MbsSample &out)
{
  if (!mbs_layout_valid(lay) || frame == nullptr)
  {
    return false;
  }
  const std::size_t need = mbs_reply_size(lay);
  if (length < need || need > out.raw.size())
  {
    return false;
  }
  if (!mbs_crc8_ok(frame, need))
  {
    return false;
  }

  MbsSample s{};
  s.raw_len = static_cast<uint8_t>(need);
  std::memcpy(s.raw.data(), frame, need);
  s.crc = frame[need - 1];

  std::size_t off = 0;
  if (lay.multi)
  {
    s.multi_raw = static_cast<uint16_t>((frame[off] << 8) | frame[off + 1]);
    s.multi_turns = static_cast<int16_t>(s.multi_raw);
    off += 2;
  }

  if (lay.single_bits == 17)
  {
    s.angle_raw = (static_cast<uint32_t>(frame[off]) << 16) |
                  (static_cast<uint32_t>(frame[off + 1]) << 8) |
                  static_cast<uint32_t>(frame[off + 2]);
    off += 3;
  }
  else
  {
    s.angle_raw = (static_cast<uint32_t>(frame[off]) << 8) |
                  static_cast<uint32_t>(frame[off + 1]);
    off += 2;
  }

  const double scale = static_cast<double>(1u << lay.single_bits);
  s.angle_deg = static_cast<double>(s.angle_raw) / scale * 360.0;
  s.angle_rad = static_cast<double>(s.angle_raw) / scale * (2.0 * kMbsPi);
  s.total_deg = static_cast<double>(s.multi_turns) * 360.0 + s.angle_deg;
  s.total_rad = static_cast<double>(s.multi_turns) * (2.0 * kMbsPi) + s.angle_rad;

  if (lay.cmd == MbsCmd::GetPosStatus)
  {
    s.has_status = true;
    s.status = mbs_decode_status(frame[off]);
    off += 1;
  }
  else if (lay.cmd == MbsCmd::GetPosTemp)
  {
    s.has_temp = true;
    const int16_t t =
        static_cast<int16_t>(static_cast<uint16_t>(frame[off + 1]) |
                             (static_cast<uint16_t>(frame[off]) << 8));
    s.temp_c = static_cast<float>(t) / 10.f;
    off += 2;
  }

  (void)off;
  out = s;
  return true;
}

inline void mbs_append_crc(uint8_t *buf, std::size_t payload_len)
{
  buf[payload_len] = mbs_crc8(buf, payload_len);
}

// Sliding-window RS485 framer: no magic header. Drop one garbage byte when the
// candidate of expected length fails CRC (covers USB-RS485 TX echo of 0x64).
class MbsRs485Framer
{
public:
  static constexpr std::size_t kMaxQueued = 256;

  explicit MbsRs485Framer(MbsLayout layout = {}) : layout_(layout) {}

  void set_layout(const MbsLayout &layout) { layout_ = layout; }
  const MbsLayout &layout() const { return layout_; }

  void feed(const uint8_t *data, std::size_t n)
  {
    if (data == nullptr)
    {
      return;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
      if (q_.size() >= kMaxQueued)
      {
        q_.pop_front();
        ++overflow_drops_;
      }
      q_.push_back(data[i]);
    }
  }

  bool pop(MbsSample &out)
  {
    const std::size_t need = mbs_reply_size(layout_);
    uint8_t tmp[8];
    while (q_.size() >= need)
    {
      auto it = q_.begin();
      for (std::size_t i = 0; i < need; ++i, ++it)
      {
        tmp[i] = *it;
      }
      if (mbs_unpack(tmp, need, layout_, out))
      {
        for (std::size_t i = 0; i < need; ++i)
        {
          q_.pop_front();
        }
        ++frames_ok_;
        return true;
      }
      q_.pop_front();
      ++garbage_drops_;
    }
    return false;
  }

  void clear() { q_.clear(); }
  std::size_t queued() const { return q_.size(); }
  std::size_t frames_ok() const { return frames_ok_; }
  std::size_t garbage_drops() const { return garbage_drops_; }
  std::size_t overflow_drops() const { return overflow_drops_; }

private:
  MbsLayout layout_{};
  std::deque<uint8_t> q_;
  std::size_t frames_ok_ = 0;
  std::size_t garbage_drops_ = 0;
  std::size_t overflow_drops_ = 0;
};

inline int mbs_bus_codec_self_test();

// Offline codec check. Returns 0 on success, else a failing step id.
inline int mbs_codec_self_test()
{
  // 17-bit 0x64: A=0x010000 → 180 deg, S=0. CRC of 01 00 00 00 is 0x79.
  {
    MbsLayout lay;
    lay.single_bits = 17;
    lay.multi = false;
    lay.cmd = MbsCmd::GetPosStatus;
    uint8_t f[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
    mbs_append_crc(f, 4);
    if (f[4] != 0x79)
    {
      return 1;
    }
    if (mbs_crc8(f, 5) != 0)
    {
      return 2;
    }
    MbsSample s{};
    if (!mbs_unpack(f, 5, lay, s))
    {
      return 3;
    }
    if (s.angle_raw != 0x010000u)
    {
      return 4;
    }
    if (s.angle_deg < 179.9 || s.angle_deg > 180.1)
    {
      return 5;
    }
    const double half_turn = kMbsPi;
    if (s.angle_rad < half_turn - 1e-9 || s.angle_rad > half_turn + 1e-9)
    {
      return 6;
    }
  }

  // 16M 0x64: M=2, A=0, S=0. Prefix garbage + TX echo 0x64 must be dropped.
  {
    MbsLayout lay;
    lay.single_bits = 16;
    lay.multi = true;
    lay.cmd = MbsCmd::GetPosStatus;
    uint8_t f[6] = {0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
    mbs_append_crc(f, 5);
    uint8_t wire[10] = {0xFF, 0x64, f[0], f[1], f[2], f[3], f[4], f[5], 0x11, 0x22};
    MbsRs485Framer fr(lay);
    fr.feed(wire, sizeof(wire));
    MbsSample s{};
    if (!fr.pop(s))
    {
      return 7;
    }
    if (s.multi_turns != 2 || s.angle_raw != 0)
    {
      return 8;
    }
    if (s.total_deg < 719.9 || s.total_deg > 720.1)
    {
      return 9;
    }
    if (fr.garbage_drops() < 2)
    {
      return 10;
    }
    MbsSample extra{};
    if (fr.pop(extra))
    {
      return 11;
    }
  }

  // Corrupt CRC must fail unpack.
  {
    MbsLayout lay;
    uint8_t f[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
    mbs_append_crc(f, 4);
    f[4] ^= 0x01;
    MbsSample s{};
    if (mbs_unpack(f, 5, lay, s))
    {
      return 12;
    }
  }

  // TX echo 0x64 + 17-bit 0x64 reply. Strip echo, then unpack 5 bytes.
  {
    MbsLayout lay;
    uint8_t f[6] = {0x64, 0x01, 0x00, 0x00, 0x00, 0x00};
    mbs_append_crc(f + 1, 4);
    const std::size_t n = mbs_strip_leading_echo(f, 6, 0x64, 5);
    if (n != 5)
    {
      return 13;
    }
    MbsSample s{};
    if (!mbs_unpack(f, n, lay, s) || s.angle_raw != 0x010000u)
    {
      return 14;
    }
    // Valid 5-byte frame that starts with 0x64 must not be stripped.
    uint8_t g[5] = {0x64, 0x00, 0x00, 0x00, 0x00};
    mbs_append_crc(g, 4);
    if (mbs_strip_leading_echo(g, 5, 0x64, 5) != 5)
    {
      return 15;
    }
    uint8_t echo_only[1] = {0x64};
    if (mbs_strip_leading_echo(echo_only, 1, 0x64, 5) != 0)
    {
      return 16;
    }
  }

  const int bus_rc = mbs_bus_codec_self_test();
  if (bus_rc != 0)
  {
    return bus_rc;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// BUS / D (高速总线). Electrical RS485, UART 8N1 @ 2.5 Mbps. Not protocol R.
// Datasheet 协议配置: 8 bit, 奇偶校验=无, 1 stop, LSB first — not 9-bit address
// mode and not termios PARENB/MARK/SPACE. Odd parity lives in request b7.
// Request 1 byte: b7 odd parity, b6:5 op, b4:0 addr. Default addr 0x1F.
// Reply: B0 = request echo, B1..Bn payload, CRC = XOR of B0..Bn (poly x^8+1).
// Get-info (op 00) 17-bit no multi (SKU ...-17-D...): n=4 → 6-byte reply
//   B0 req | B1 S | B2 A0 | B3 A1 | B4 A2 | B5 CRC   (A little-endian)
// Op 0b11 has no reply — that is why TX 0x64 (b6:5=11) is silent on a D SKU.
// ---------------------------------------------------------------------------

enum class MbsBusOp : uint8_t
{
  GetInfo = 0,   // b6:5 = 00
  SetZero = 1,   // b6:5 = 01
  SetAddr = 2,   // b6:5 = 10
};

enum class MbsIface : uint8_t
{
  Rs485R = 0,
  BusD = 1,
};

struct MbsBusLayout
{
  uint8_t single_bits = 17;  // 16 or 17
  uint8_t multi_bits = 0;    // 0, 8 (XXM1-D), 16 (XXM2-D)
  MbsBusOp op = MbsBusOp::GetInfo;
  uint8_t addr = kMbsBusAddrDefault;
};

inline bool mbs_bus_layout_valid(const MbsBusLayout &lay)
{
  const bool bits_ok = lay.single_bits == 16 || lay.single_bits == 17;
  const bool multi_ok = lay.multi_bits == 0 || lay.multi_bits == 8 || lay.multi_bits == 16;
  const bool addr_ok = (lay.addr & static_cast<uint8_t>(~kMbsBusAddrMask)) == 0;
  return bits_ok && multi_ok && addr_ok;
}

// Odd parity of bits 0..6 stored in b7 so the whole byte has an odd popcount.
inline uint8_t mbs_bus_with_odd_parity(uint8_t low7)
{
  uint8_t b = static_cast<uint8_t>(low7 & 0x7Fu);
  uint8_t x = b;
  x ^= static_cast<uint8_t>(x >> 4);
  x ^= static_cast<uint8_t>(x >> 2);
  x ^= static_cast<uint8_t>(x >> 1);
  if ((x & 1u) == 0)
  {
    b |= 0x80u;
  }
  return b;
}

inline bool mbs_bus_odd_parity_ok(uint8_t b)
{
  uint8_t x = b;
  x ^= static_cast<uint8_t>(x >> 4);
  x ^= static_cast<uint8_t>(x >> 2);
  x ^= static_cast<uint8_t>(x >> 1);
  return (x & 1u) != 0;
}

inline uint8_t mbs_bus_request(MbsBusOp op, uint8_t addr)
{
  const uint8_t low7 =
      static_cast<uint8_t>(((static_cast<uint8_t>(op) & 0x03u) << 5) |
                           (addr & kMbsBusAddrMask));
  return mbs_bus_with_odd_parity(low7);
}

inline uint8_t mbs_bus_request(const MbsBusLayout &lay)
{
  return mbs_bus_request(lay.op, lay.addr);
}

// Datasheet appendix CRC-8 (x^8+1): XOR of all bytes. Table == result.
inline uint8_t mbs_bus_crc8(const uint8_t *data, std::size_t length)
{
  uint8_t x = 0;
  if (data == nullptr)
  {
    return 0;
  }
  for (std::size_t i = 0; i < length; ++i)
  {
    x = static_cast<uint8_t>(x ^ data[i]);
  }
  return x;
}

inline bool mbs_bus_crc8_ok(const uint8_t *frame, std::size_t length)
{
  if (frame == nullptr || length < 2)
  {
    return false;
  }
  return mbs_bus_crc8(frame, length) == 0;
}

inline void mbs_bus_append_crc(uint8_t *buf, std::size_t payload_len)
{
  buf[payload_len] = mbs_bus_crc8(buf, payload_len);
}

// Get-info payload n (B1..Bn), excluding B0 and CRC.
inline std::size_t mbs_bus_data_n(const MbsBusLayout &lay)
{
  std::size_t n = 1;  // S
  n += lay.single_bits > 16 ? 3u : 2u;
  n += lay.multi_bits / 8u;
  return n;
}

// Full reply: B0 + B1..Bn + CRC.
inline std::size_t mbs_bus_reply_size(const MbsBusLayout &lay)
{
  return 1u + mbs_bus_data_n(lay) + 1u;
}

inline std::size_t mbs_bus_strip_leading_echo(uint8_t *buf, std::size_t n, uint8_t req,
                                             std::size_t reply_n)
{
  if (buf == nullptr || n == 0 || buf[0] != req)
  {
    return n;
  }
  if (n >= reply_n && mbs_bus_crc8_ok(buf, reply_n))
  {
    return n;
  }
  if (n == 1 || n >= reply_n + 1)
  {
    std::memmove(buf, buf + 1, n - 1);
    return n - 1;
  }
  return n;
}

inline bool mbs_bus_unpack(const uint8_t *frame, std::size_t length, const MbsBusLayout &lay,
                           MbsSample &out)
{
  if (!mbs_bus_layout_valid(lay) || frame == nullptr)
  {
    return false;
  }
  const std::size_t need = mbs_bus_reply_size(lay);
  if (length < need || need > out.raw.size())
  {
    return false;
  }
  if (!mbs_bus_crc8_ok(frame, need))
  {
    return false;
  }
  const uint8_t req = mbs_bus_request(lay);
  if (frame[0] != req)
  {
    return false;
  }

  MbsSample s{};
  s.raw_len = static_cast<uint8_t>(need);
  std::memcpy(s.raw.data(), frame, need);
  s.crc = frame[need - 1];

  std::size_t off = 1;
  s.has_status = true;
  s.status = mbs_decode_status(frame[off]);
  off += 1;

  if (lay.single_bits == 17)
  {
    s.angle_raw = static_cast<uint32_t>(frame[off]) |
                  (static_cast<uint32_t>(frame[off + 1]) << 8) |
                  (static_cast<uint32_t>(frame[off + 2]) << 16);
    s.angle_raw &= 0x1FFFFu;
    off += 3;
  }
  else
  {
    s.angle_raw = static_cast<uint32_t>(frame[off]) |
                  (static_cast<uint32_t>(frame[off + 1]) << 8);
    off += 2;
  }

  if (lay.multi_bits == 8)
  {
    s.multi_raw = frame[off];
    s.multi_turns = static_cast<int8_t>(frame[off]);
    off += 1;
  }
  else if (lay.multi_bits == 16)
  {
    s.multi_raw = static_cast<uint16_t>(frame[off] | (frame[off + 1] << 8));
    s.multi_turns = static_cast<int16_t>(s.multi_raw);
    off += 2;
  }

  const double scale = static_cast<double>(1u << lay.single_bits);
  s.angle_deg = static_cast<double>(s.angle_raw) / scale * 360.0;
  s.angle_rad = static_cast<double>(s.angle_raw) / scale * (2.0 * kMbsPi);
  s.total_deg = static_cast<double>(s.multi_turns) * 360.0 + s.angle_deg;
  s.total_rad = static_cast<double>(s.multi_turns) * (2.0 * kMbsPi) + s.angle_rad;

  (void)off;
  out = s;
  return true;
}

class MbsBusFramer
{
public:
  static constexpr std::size_t kMaxQueued = 256;

  explicit MbsBusFramer(MbsBusLayout layout = {}) : layout_(layout) {}

  void set_layout(const MbsBusLayout &layout) { layout_ = layout; }
  const MbsBusLayout &layout() const { return layout_; }

  void feed(const uint8_t *data, std::size_t n)
  {
    if (data == nullptr)
    {
      return;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
      if (q_.size() >= kMaxQueued)
      {
        q_.pop_front();
        ++overflow_drops_;
      }
      q_.push_back(data[i]);
    }
  }

  bool pop(MbsSample &out)
  {
    const std::size_t need = mbs_bus_reply_size(layout_);
    uint8_t tmp[8];
    while (q_.size() >= need)
    {
      auto it = q_.begin();
      for (std::size_t i = 0; i < need; ++i, ++it)
      {
        tmp[i] = *it;
      }
      if (mbs_bus_unpack(tmp, need, layout_, out))
      {
        for (std::size_t i = 0; i < need; ++i)
        {
          q_.pop_front();
        }
        ++frames_ok_;
        return true;
      }
      q_.pop_front();
      ++garbage_drops_;
    }
    return false;
  }

  void clear() { q_.clear(); }
  std::size_t queued() const { return q_.size(); }
  std::size_t frames_ok() const { return frames_ok_; }
  std::size_t garbage_drops() const { return garbage_drops_; }
  std::size_t overflow_drops() const { return overflow_drops_; }

private:
  MbsBusLayout layout_{};
  std::deque<uint8_t> q_;
  std::size_t frames_ok_ = 0;
  std::size_t garbage_drops_ = 0;
  std::size_t overflow_drops_ = 0;
};

// Offline BUS-D checks from datasheet v2.7 (no numeric example in the PDF;
// vectors are built from the request/CRC/layout tables).
inline int mbs_bus_codec_self_test()
{
  // Default get-info @ 0x1F: bits 0-6 already odd → TX 0x1F.
  if (mbs_bus_request(MbsBusOp::GetInfo, kMbsBusAddrDefault) != 0x1F)
  {
    return 20;
  }
  if (!mbs_bus_odd_parity_ok(0x1F))
  {
    return 21;
  }
  // Addr 0x00: low7 even → parity bit set.
  if (mbs_bus_request(MbsBusOp::GetInfo, 0x00) != 0x80)
  {
    return 22;
  }
  // Addr 0x01: one 1 already odd.
  if (mbs_bus_request(MbsBusOp::GetInfo, 0x01) != 0x01)
  {
    return 23;
  }
  // Set-zero op=01, addr 0x1F → 0b00111111=0x3F, six ones → parity 0xBF.
  if (mbs_bus_request(MbsBusOp::SetZero, kMbsBusAddrDefault) != 0xBF)
  {
    return 24;
  }
  // Set-addr op=10, addr 0x1F → 0b01011111=0x5F, even ones → 0xDF.
  if (mbs_bus_request(MbsBusOp::SetAddr, kMbsBusAddrDefault) != 0xDF)
  {
    return 24;
  }

  // 17-bit get-info, 180 deg, S=0. A = 0x010000 little-endian A0 A1 A2.
  // TX 1F. RX 1F 00 00 00 01 CRC=1F^01=1E.
  {
    MbsBusLayout lay;
    lay.single_bits = 17;
    lay.multi_bits = 0;
    lay.op = MbsBusOp::GetInfo;
    lay.addr = kMbsBusAddrDefault;
    if (mbs_bus_reply_size(lay) != 6)
    {
      return 25;
    }
    uint8_t f[6] = {0x1F, 0x00, 0x00, 0x00, 0x01, 0x00};
    mbs_bus_append_crc(f, 5);
    if (f[5] != 0x1E)
    {
      return 26;
    }
    if (!mbs_bus_crc8_ok(f, 6))
    {
      return 27;
    }
    MbsSample s{};
    if (!mbs_bus_unpack(f, 6, lay, s) || s.angle_raw != 0x010000u)
    {
      return 28;
    }
    if (s.angle_deg < 179.9 || s.angle_deg > 180.1)
    {
      return 29;
    }
    if (!s.has_status || s.status.raw != 0)
    {
      return 30;
    }
  }

  // TX echo 0x1F + 6-byte reply. Strip / sliding window must still unpack.
  {
    MbsBusLayout lay;
    uint8_t f[7] = {0x1F, 0x1F, 0x00, 0x00, 0x00, 0x01, 0x00};
    mbs_bus_append_crc(f + 1, 5);
    const std::size_t n = mbs_bus_strip_leading_echo(f, 7, 0x1F, 6);
    if (n != 6)
    {
      return 31;
    }
    MbsSample s{};
    if (!mbs_bus_unpack(f, n, lay, s) || s.angle_raw != 0x010000u)
    {
      return 32;
    }
    uint8_t wire[9] = {0xFF, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x01, 0x1E, 0x22};
    MbsBusFramer fr(lay);
    fr.feed(wire, sizeof(wire));
    MbsSample g{};
    if (!fr.pop(g) || g.angle_raw != 0x010000u)
    {
      return 33;
    }
    if (fr.garbage_drops() < 2)
    {
      return 34;
    }
  }

  // Valid 6-byte frame starting with 0x1F must not be stripped as echo.
  {
    uint8_t g[6] = {0x1F, 0x00, 0x00, 0x00, 0x01, 0x1E};
    if (mbs_bus_strip_leading_echo(g, 6, 0x1F, 6) != 6)
    {
      return 35;
    }
  }

  // Wrong B0 (other address) must fail.
  {
    MbsBusLayout lay;
    uint8_t f[6] = {0x01, 0x00, 0x00, 0x00, 0x01, 0x00};
    mbs_bus_append_crc(f, 5);
    MbsSample s{};
    if (mbs_bus_unpack(f, 6, lay, s))
    {
      return 36;
    }
  }

  // 16-bit no multi: n=3 data → 5-byte frame. S A0 A1.
  {
    MbsBusLayout lay;
    lay.single_bits = 16;
    if (mbs_bus_reply_size(lay) != 5)
    {
      return 37;
    }
    uint8_t f[5] = {0x1F, 0x00, 0x00, 0x00, 0x00};
    mbs_bus_append_crc(f, 4);
    MbsSample s{};
    if (!mbs_bus_unpack(f, 5, lay, s) || s.angle_raw != 0)
    {
      return 38;
    }
  }
  return 0;
}

}  // namespace biped_hw
