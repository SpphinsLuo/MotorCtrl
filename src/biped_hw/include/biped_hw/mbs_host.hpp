// biped_hw — mbs_host.hpp
// BUS-D GetInfo poll / SetZero TX on an already-open SerialPort (P4 sequential map).
#pragma once

#include "biped_hw/mbs_codec.hpp"
#include "biped_hw/serial_port.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace biped_hw
{

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
  uint8_t view[16];
  std::size_t view_n = static_cast<std::size_t>(n) < sizeof(view)
                           ? static_cast<std::size_t>(n)
                           : sizeof(view);
  std::memcpy(view, raw, view_n);
  view_n = mbs_bus_strip_leading_echo(view, view_n, tx, reply_n);
  return view_n >= reply_n && mbs_bus_unpack(view, view_n, lay, out);
}

// Writes encoder origin at current angle (device SetZero). Same SerialPort +
// RTS-DE TX as mbs_bus_poll_once. Not yaml zero_rad (that stays host-side).
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

}  // namespace biped_hw
