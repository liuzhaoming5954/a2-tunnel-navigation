#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace a2_platform {
using SteadyTime = std::chrono::steady_clock::time_point;

struct RawState {
  uint64_t sequence = 0;
  SteadyTime received_at;
  std::optional<double> battery_percent;
  // Measured planar base_link vx, vy, yaw_rate. Never a command.
  std::optional<std::array<double, 3>> body_velocity;
  std::optional<uint32_t> mode;
};

class StateSource {
 public:
  virtual ~StateSource() = default;
  virtual std::optional<RawState> latest() = 0;
};

class MockStateSource final : public StateSource {
 public:
  explicit MockStateSource(
      std::function<SteadyTime()> clock,
      std::optional<std::chrono::milliseconds> drop_after = std::nullopt);
  std::optional<RawState> latest() override;

 private:
  std::function<SteadyTime()> clock_;
  SteadyTime started_at_;
  std::optional<std::chrono::milliseconds> drop_after_;
  std::optional<RawState> cached_;
  uint64_t sequence_ = 0;
};

struct UnitreeConfig {
  std::string network_interface;
  int domain_id = 0;
};

// Defined only when the optional receive-only SDK adapter is compiled.
std::unique_ptr<StateSource> make_unitree_state_source(const UnitreeConfig& config);
}  // namespace a2_platform
