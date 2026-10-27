#include "mbs_encoder/mbs_codec.hpp"
#include "mbs_encoder/mbs_host.hpp"
#include "mbs_encoder/mbs_map.hpp"
#include "mbs_encoder/serial_port.hpp"

#include <rclcpp/rclcpp.hpp>
#include <mbs_encoder/msg/mbs_state.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

class MbsEncoderNode final : public rclcpp::Node
{
public:
  MbsEncoderNode()
  : Node("mbs_encoder_node"),
    timeout_ms_(declare_parameter<int>("timeout_ms", 8)),
    period_ms_(declare_parameter<int>("period_ms", 10)),
    topic_(declare_parameter<std::string>("topic", "/mbs/state")),
    port_(declare_parameter<std::string>("port", ""))
  {
    biped_hw::MbsEncoderConfig cfg;
    std::string err;
    if (!biped_hw::load_mbs_encoder_config(cfg, err))
    {
      throw std::runtime_error(err);
    }
    if (port_.empty())
    {
      port_ = cfg.port;
    }
    serial_ = std::make_unique<biped_hw::SerialPort>(
        port_, B2500000, 2, biped_hw::kMbsBusBaudDefault);
    for (const auto &entry : cfg.encoders)
    {
      entries_.push_back(entry);
      timeout_counts_.push_back(0);
    }

    publisher_ = create_publisher<mbs_encoder::msg::MbsState>(topic_, 10);
    if (!serial_->open())
    {
      RCLCPP_ERROR(get_logger(), "cannot open MBS port %s: %s", port_.c_str(),
                   serial_->last_error().c_str());
    }
    else if (biped_hw::tty_driver_name(port_) != "pl2303")
    {
      rts_de_ = false;
    }
    RCLCPP_INFO(get_logger(), "MBS node port=%s topic=%s period=%dms timeout=%dms",
                port_.c_str(), topic_.c_str(), period_ms_, timeout_ms_);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms_), std::bind(&MbsEncoderNode::poll, this));
  }

private:
  void poll()
  {
    if (serial_ == nullptr || !serial_->is_open())
    {
      publish_invalid();
      return;
    }

    struct Sample
    {
      uint8_t addr = 0;
      std::string name;
      double angle_rad = 0.0;
      double total_rad = 0.0;
      uint32_t raw = 0;
      bool valid = false;
      uint8_t status_raw = 0;
      uint32_t timeout_count = 0;
    };
    Sample samples[biped_hw::kMbsEncoderCount]{};
    bool any_valid = false;
    for (std::size_t i = 0; i < entries_.size(); ++i)
    {
      const auto &entry = entries_[i];
      biped_hw::MbsSample sample{};
      const bool ok = biped_hw::mbs_bus_poll_once(
          *serial_, entry.addr, timeout_ms_, rts_de_, sample);
      samples[i].addr = entry.addr;
      samples[i].name = entry.name;
      samples[i].valid = ok;
      if (ok)
      {
        samples[i].angle_rad = sample.angle_rad;
        samples[i].total_rad = sample.total_rad;
        samples[i].raw = sample.angle_raw;
        samples[i].status_raw = sample.has_status ? sample.status.raw : 0;
        timeout_counts_[i] = 0;
        any_valid = true;
      }
      else
      {
        ++timeout_counts_[i];
      }
      samples[i].timeout_count = timeout_counts_[i];
    }

    mbs_encoder::msg::MbsState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "mbs";
    msg.sequence = ++sequence_;
    msg.addr.reserve(entries_.size());
    msg.name.reserve(entries_.size());
    msg.angle_rad.reserve(entries_.size());
    msg.total_rad.reserve(entries_.size());
    msg.raw.reserve(entries_.size());
    msg.valid.reserve(entries_.size());
    msg.status_raw.reserve(entries_.size());
    msg.timeout_count.reserve(entries_.size());
    for (std::size_t i = 0; i < entries_.size(); ++i)
    {
      msg.addr.push_back(samples[i].addr);
      msg.name.push_back(samples[i].name);
      msg.angle_rad.push_back(samples[i].angle_rad);
      msg.total_rad.push_back(samples[i].total_rad);
      msg.raw.push_back(samples[i].raw);
      msg.valid.push_back(samples[i].valid);
      msg.status_raw.push_back(samples[i].status_raw);
      msg.timeout_count.push_back(samples[i].timeout_count);
    }
    publisher_->publish(msg);
    if (!any_valid)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no valid MBS samples");
    }
  }

  void publish_invalid()
  {
    mbs_encoder::msg::MbsState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "mbs";
    msg.sequence = ++sequence_;
    for (const auto &entry : entries_)
    {
      msg.addr.push_back(entry.addr);
      msg.name.push_back(entry.name);
      msg.angle_rad.push_back(0.0);
      msg.total_rad.push_back(0.0);
      msg.raw.push_back(0);
      msg.valid.push_back(false);
      msg.status_raw.push_back(0);
      msg.timeout_count.push_back(++timeout_counts_[&entry - entries_.data()]);
    }
    publisher_->publish(msg);
  }

  int timeout_ms_;
  int period_ms_;
  std::string topic_;
  std::string port_;
  bool rts_de_ = true;
  std::unique_ptr<biped_hw::SerialPort> serial_;
  std::vector<biped_hw::MbsMapEntry> entries_;
  std::vector<std::uint32_t> timeout_counts_;
  std::uint32_t sequence_ = 0;
  rclcpp::Publisher<mbs_encoder::msg::MbsState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<MbsEncoderNode>());
  }
  catch (const std::exception &e)
  {
    std::fprintf(stderr, "mbs_encoder_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
