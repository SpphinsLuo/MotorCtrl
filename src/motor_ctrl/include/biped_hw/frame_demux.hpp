// biped_hw — frame_demux.hpp
// 单总线 RX 分流：达妙反馈通常是 Master 0x02，按 payload D[0] 低 4 位识别 ESC_ID。
// 部分固件 MST_ID=从站 CAN（如 12），回包 can_id==ESC 而 D[0] 低位不一定是 12。
// Motorevo 回包可能走从站 CAN 8/14，也可能走 Master 0x02；take_motorevo 两者都收。
// P1/P2 按单个 ESC 取包；P3 用 take_wanted_mit，避免误丢同周期其它电机的反馈。
#pragma once

// One-bus RX demux. Damiao feedback is usually Master 0x02; identify ESC from
// D[0] low nibble. Also accept slave-id feedback (rx.can_id == ESC, e.g. 12).
// Do not use the official addMotor dual-key (Slave+Master) map.

#include "biped_hw/dm_codec.hpp"
#include "biped_hw/motorevo_codec.hpp"
#include "biped_hw/types.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>

namespace biped_hw
{

class FrameDemux
{
public:
  static constexpr std::size_t kMaxQueued = 64;

  void push(const CanRxFrame &rx)
  {
    if (q_.size() >= kMaxQueued)
    {
      q_.pop_front();
      ++overflow_drops_;
    }
    q_.push_back(rx);
  }

  bool take_mit(uint8_t esc_id, const LimitParam &lim, DmFeedback &out)
  {
    for (auto it = q_.begin(); it != q_.end();)
    {
      if (is_dm_register_payload(it->data, it->dlc))
      {
        ++it;
        continue;
      }
      DmFeedback fb{};
      if (!unpack_mit_feedback(it->data, it->dlc, it->can_id, lim, fb))
      {
        ++it;
        continue;
      }
      if (fb.esc_id != esc_id)
      {
        ++other_esc_;
        it = q_.erase(it);
        continue;
      }
      out = fb;
      q_.erase(it);
      return true;
    }
    return false;
  }

  bool take_register(uint16_t esc_id, uint8_t rid, DmRegisterReply &out)
  {
    for (auto it = q_.begin(); it != q_.end(); ++it)
    {
      DmRegisterReply r{};
      if (!unpack_register(it->data, it->dlc, r))
      {
        continue;
      }
      if (r.esc_id != esc_id || r.rid != rid)
      {
        continue;
      }
      out = r;
      q_.erase(it);
      return true;
    }
    return false;
  }

  // First MIT frame whose ESC_ID is wanted. Other motors stay queued (unlike take_mit).
  // Match Master-2 + D[0] nibble, or slave-id feedback (rx.can_id == wanted ESC).
  bool take_wanted_mit(const bool wanted[16], const LimitParam lims[16], DmFeedback &out)
  {
    for (auto it = q_.begin(); it != q_.end(); ++it)
    {
      if (it->dlc < 6 || is_dm_register_payload(it->data, it->dlc))
      {
        continue;
      }
      const uint8_t nibble = static_cast<uint8_t>(it->data[0] & 0x0F);
      uint8_t esc = 0xFF;
      if (wanted[nibble])
      {
        esc = nibble;
      }
      else if (it->can_id < 16u && wanted[static_cast<uint8_t>(it->can_id)])
      {
        // Damiao MST_ID programmed equal to ESC_ID: USB-CAN reports can_id=12
        // (or 6), not Master 2. D[0] nibble may not equal ESC.
        esc = static_cast<uint8_t>(it->can_id);
      }
      else
      {
        continue;
      }
      DmFeedback fb{};
      if (!unpack_mit_feedback(it->data, it->dlc, it->can_id, lims[esc], fb))
      {
        continue;
      }
      fb.esc_id = esc;
      out = fb;
      q_.erase(it);
      return true;
    }
    return false;
  }

  // Motorevo (and similar): match USB-CAN can_id, leave Damiao Master-2 frames.
  bool take_by_can_id(uint32_t can_id, CanRxFrame &out)
  {
    for (auto it = q_.begin(); it != q_.end(); ++it)
    {
      if (it->can_id != can_id)
      {
        continue;
      }
      out = *it;
      q_.erase(it);
      return true;
    }
    return false;
  }

  // Prefer slave CAN ID (8/14). Also accept Master 0x02 unless it looks like a
  // Damiao MIT reply (ESC nibble 3–7 / 9–13).
  bool take_motorevo(uint8_t slave_id, MotorevoFeedback &out)
  {
    auto is_left_damiao_esc = [](uint8_t nibble) {
      return nibble >= 3 && nibble <= 7;
    };
    auto is_right_damiao_esc = [](uint8_t nibble) {
      return nibble >= 9 && nibble <= 13;
    };
    auto consider = [&](const CanRxFrame &rx) -> bool {
      if (rx.dlc < 6 || is_dm_register_payload(rx.data, rx.dlc))
      {
        return false;
      }
      if (rx.can_id == slave_id)
      {
        return true;
      }
      if (rx.can_id != kMasterId)
      {
        return false;
      }
      const uint8_t nibble = static_cast<uint8_t>(rx.data[0] & 0x0F);
      if (nibble == slave_id || is_motorevo_can_id(nibble))
      {
        return true;
      }
      if (is_left_damiao_esc(nibble) || is_right_damiao_esc(nibble))
      {
        return false;
      }
      return true;
    };

    for (int pass = 0; pass < 2; ++pass)
    {
      const uint32_t want = (pass == 0) ? static_cast<uint32_t>(slave_id)
                                        : static_cast<uint32_t>(kMasterId);
      for (auto it = q_.begin(); it != q_.end(); ++it)
      {
        if (it->can_id != want || !consider(*it))
        {
          continue;
        }
        MotorevoFeedback fb{};
        if (!unpack_motorevo_feedback(*it, fb))
        {
          continue;
        }
        out = fb;
        q_.erase(it);
        return true;
      }
    }
    return false;
  }

  std::size_t size() const { return q_.size(); }
  std::size_t overflow_drops() const { return overflow_drops_; }
  std::size_t other_esc_seen() const { return other_esc_; }

  void clear()
  {
    q_.clear();
  }

private:
  std::deque<CanRxFrame> q_;
  std::size_t overflow_drops_ = 0;
  std::size_t other_esc_ = 0;
};

}  // namespace biped_hw
