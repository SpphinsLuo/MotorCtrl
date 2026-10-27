// biped_hw — mbs_host.hpp
// BUS-D GetInfo poll / SetZero TX on an already-open SerialPort (P4 sequential map).
#pragma once

#include "mbs_encoder/mbs_codec.hpp"
#include "mbs_encoder/serial_port.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <unistd.h>

namespace biped_hw
{

inline bool mbs_bus_tx_rx(SerialPort &serial, uint8_t tx, std::size_t reply_n, int timeout_ms,
                          bool rts_de, uint8_t *view, std::size_t view_cap, std::size_t &view_n)
{
  if (view == nullptr || view_cap < reply_n)
  {
    return false;
  }
  serial.flush_input();
  if (rts_de)
  {
    serial.set_rts(true);
  }
  if (serial.send(&tx, 1) != 1)
  {
    if (rts_de)
    {
      serial.set_rts(false);
    }
    return false;
  }
  serial.drain_output();
  if (rts_de)
  {
    serial.set_rts(false);
  }
  uint8_t raw[16];
  const ssize_t n = serial.recv_n(raw, reply_n + 1, timeout_ms);
  if (n <= 0)
  {
    return false;
  }
  view_n = static_cast<std::size_t>(n) < view_cap ? static_cast<std::size_t>(n) : view_cap;
  std::memcpy(view, raw, view_n);
  view_n = mbs_bus_strip_leading_echo(view, view_n, tx, reply_n);
  return view_n >= reply_n;
}

inline bool mbs_bus_poll_once(SerialPort &serial, uint8_t addr, int timeout_ms, bool rts_de,
                              MbsSample &out)
{
  MbsBusLayout lay;
  lay.single_bits = 17;
  lay.multi_bits = 0;
  lay.op = MbsBusOp::GetInfo;
  lay.addr = addr;
  const uint8_t tx = mbs_bus_request(lay);
  const std::size_t reply_n = mbs_bus_reply_size(lay);
  uint8_t view[16];
  std::size_t view_n = 0;
  if (!mbs_bus_tx_rx(serial, tx, reply_n, timeout_ms, rts_de, view, sizeof(view), view_n))
  {
    return false;
  }
  return mbs_bus_unpack(view, view_n, lay, out);
}

// Single 01 TX. Datasheet SetZero needs mbs_bus_set_zero_sequence, not this alone.
inline bool mbs_bus_send_set_zero(SerialPort &serial, uint8_t addr, bool rts_de)
{
  const uint8_t tx = mbs_bus_request(MbsBusOp::SetZero, addr);
  serial.flush_input();
  if (rts_de)
  {
    serial.set_rts(true);
  }
  if (serial.send(&tx, 1) != 1)
  {
    if (rts_de)
    {
      serial.set_rts(false);
    }
    return false;
  }
  serial.drain_output();
  if (rts_de)
  {
    serial.set_rts(false);
  }
  return true;
}

struct MbsSetZeroResult
{
  int pairs = 0;
  uint8_t last_c = 0;
  uint8_t last_s = 0;
  bool committed = false;
};

// Alternate GetInfo/SetZero kMbsBusSetZeroPairs times. C==10 then wait 50 ms.
inline bool mbs_bus_set_zero_sequence(SerialPort &serial, uint8_t addr, int timeout_ms,
                                      bool rts_de, MbsSetZeroResult &st)
{
  st = {};
  for (int i = 0; i < kMbsBusSetZeroPairs; ++i)
  {
    MbsSample info{};
    if (!mbs_bus_poll_once(serial, addr, timeout_ms, rts_de, info))
    {
      return false;
    }
    const uint8_t tx = mbs_bus_request(MbsBusOp::SetZero, addr);
    uint8_t view[16];
    std::size_t view_n = 0;
    if (!mbs_bus_tx_rx(serial, tx, kMbsBusSetAckSize, timeout_ms, rts_de, view, sizeof(view),
                       view_n))
    {
      return false;
    }
    MbsSetAck ack{};
    if (!mbs_bus_unpack_set_ack(view, view_n, addr, ack))
    {
      return false;
    }
    st.pairs = i + 1;
    st.last_c = ack.count;
    st.last_s = ack.status;
    if (ack.count == 10)
    {
      st.committed = true;
      usleep(static_cast<useconds_t>(kMbsBusSetZeroExecMs * 1000));
      return true;
    }
  }
  return false;
}

}  // namespace biped_hw
