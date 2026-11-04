#include "a2_platform/state_validation.hpp"

#include <a2_interfaces/msg/platform_state.hpp>
#include <a2_interfaces/srv/get_platform_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace a2_platform {
class ReadonlyNode final : public rclcpp::Node {
 public:
  ReadonlyNode() : Node("a2_platform_readonly") {
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    source_name_ = declare_parameter<std::string>("source", "mock", immutable);
    robot_id_ = declare_parameter<std::string>("robot_id", "mock-a2", immutable);
    const auto nic = declare_parameter<std::string>("network_interface", "", immutable);
    const auto domain = declare_parameter<int>("unitree_domain_id", 0, immutable);
    const auto period = declare_parameter<int>("publish_period_ms", 100, immutable);
    const auto timeout = declare_parameter<int>("stale_timeout_ms", 500, immutable);
    const auto drop = declare_parameter<double>("mock_drop_after_s", -1.0, immutable);
    if (robot_id_.empty() || period <= 0 || timeout <= 0 || !std::isfinite(drop) ||
        (drop < 0.0 && drop != -1.0) || drop > 86400.0) {
      throw std::invalid_argument("invalid identity, timing or mock-drop configuration");
    }
    stale_timeout_ = std::chrono::milliseconds(timeout);
    if (source_name_ == "mock") {
      std::optional<std::chrono::milliseconds> drop_after;
      if (drop >= 0.0) {
        drop_after = std::chrono::milliseconds(static_cast<int64_t>(drop * 1000));
      }
      source_ = std::make_unique<MockStateSource>(
          [] { return std::chrono::steady_clock::now(); }, drop_after);
    } else if (source_name_ == "unitree") {
      if (robot_id_ == "mock-a2" || nic.empty() || domain < 0) {
        throw std::invalid_argument("unitree requires explicit robot_id, network_interface and valid domain");
      }
#ifdef A2_ENABLE_UNITREE_SDK
      source_ = make_unitree_state_source(UnitreeConfig{nic, domain});
#else
      throw std::runtime_error("Unitree source was not compiled; no fallback to mock");
#endif
    } else {
      throw std::invalid_argument("source must be mock or unitree");
    }
    std::random_device random;
    session_ = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
               "-" + std::to_string(random());
    state_pub_ = create_publisher<a2_interfaces::msg::PlatformState>("/a2/platform/state", 10);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    service_ = create_service<a2_interfaces::srv::GetPlatformState>(
        "/a2/platform/get_state",
        [this](const std::shared_ptr<a2_interfaces::srv::GetPlatformState::Request>,
               std::shared_ptr<a2_interfaces::srv::GetPlatformState::Response> response) {
          // Reevaluate freshness at query time without obtaining a new source sample.
          response->state = message(check_state(cached_, std::chrono::steady_clock::now(), stale_timeout_));
          response->available = cached_.has_value();
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(period), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Receive-only source=%s; motion_capable=false", source_name_.c_str());
  }

 private:
  a2_interfaces::msg::PlatformState message(const CheckedState& checked) {
    a2_interfaces::msg::PlatformState output;
    output.schema_version = 1;
    output.header.stamp = now();  // Publication clock, NOT the vendor acquisition time.
    output.header.frame_id = "base_link";
    output.robot_id = robot_id_;
    output.source = source_name_ == "mock" ? "mock" : "unitree_sdk2";
    output.source_session_id = session_;
    output.valid = checked.valid;
    output.simulated = source_name_ == "mock";
    output.motion_capable = false;
    output.age_ms = checked.age_ms;
    output.battery_valid = checked.battery_valid;
    output.velocity_valid = checked.velocity_valid;
    output.mode_valid = checked.mode_valid;
    output.reason = checked.reason;
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    output.battery_percent = std::numeric_limits<float>::quiet_NaN();
    output.body_velocity.linear.x = output.body_velocity.linear.y = output.body_velocity.linear.z = nan;
    output.body_velocity.angular.x = output.body_velocity.angular.y = output.body_velocity.angular.z = nan;
    if (checked.sample) {
      const auto& raw = *checked.sample;
      output.sample_seq = raw.sequence;
      output.received_mono_ns = static_cast<uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(raw.received_at.time_since_epoch()).count());
      if (checked.battery_valid) {
        output.battery_percent = static_cast<float>(*raw.battery_percent);
      }
      if (checked.velocity_valid) {
        output.body_velocity.linear.x = (*raw.body_velocity)[0];
        output.body_velocity.linear.y = (*raw.body_velocity)[1];
        output.body_velocity.angular.z = (*raw.body_velocity)[2];
      }
      if (checked.mode_valid) {
        output.mode = *raw.mode;
      }
    }
    return output;
  }

  void tick() {
    cached_ = source_->latest();
    const auto checked = check_state(cached_, std::chrono::steady_clock::now(), stale_timeout_);
    const auto state = message(checked);
    state_pub_->publish(state);
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header = state.header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "a2_platform/readonly";
    status.hardware_id = robot_id_;
    status.level = checked.valid ? diagnostic_msgs::msg::DiagnosticStatus::OK :
                                  diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = checked.reason;
    auto append = [&status](const std::string& key, const std::string& value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(item);
    };
    append("source", state.source);
    append("simulated", state.simulated ? "true" : "false");
    append("motion_capable", "false");
    append("sample_seq", std::to_string(state.sample_seq));
    append("age_ms", std::to_string(state.age_ms));
    diagnostics.status.push_back(status);
    diagnostics_pub_->publish(diagnostics);
  }

  std::string source_name_, robot_id_, session_;
  std::chrono::milliseconds stale_timeout_{500};
  std::unique_ptr<StateSource> source_;
  std::optional<RawState> cached_;
  rclcpp::Publisher<a2_interfaces::msg::PlatformState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<a2_interfaces::srv::GetPlatformState>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace a2_platform

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<a2_platform::ReadonlyNode>());
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("a2_platform_readonly"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
