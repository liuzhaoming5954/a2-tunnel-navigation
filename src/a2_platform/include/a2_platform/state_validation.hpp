#pragma once

#include "a2_platform/state_source.hpp"

namespace a2_platform {
struct CheckedState {
  std::optional<RawState> sample;
  bool valid = false;
  bool battery_valid = false;
  bool velocity_valid = false;
  bool mode_valid = false;
  uint64_t age_ms = 0;
  std::string reason = "NO_DATA";
};

CheckedState check_state(std::optional<RawState> sample, SteadyTime now,
                         std::chrono::milliseconds stale_timeout);
}  // namespace a2_platform
