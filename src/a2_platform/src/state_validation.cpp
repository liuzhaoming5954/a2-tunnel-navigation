#include "a2_platform/state_validation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace a2_platform {
MockStateSource::MockStateSource(
    std::function<SteadyTime()> clock,
    std::optional<std::chrono::milliseconds> drop_after)
    : clock_(std::move(clock)), drop_after_(drop_after) {
  if (!clock_ || (drop_after_ && drop_after_->count() < 0)) {
    throw std::invalid_argument("mock clock and drop interval must be valid");
  }
  started_at_ = clock_();
}

std::optional<RawState> MockStateSource::latest() {
  const auto now = clock_();
  if (drop_after_ && now - started_at_ >= *drop_after_) {
    return cached_;
  }
  cached_ = RawState{++sequence_, now, 82.0,
                     std::array<double, 3>{0.0, 0.0, 0.0}, uint32_t{0}};
  return cached_;
}

CheckedState check_state(std::optional<RawState> sample, SteadyTime now,
                         std::chrono::milliseconds stale_timeout) {
  if (stale_timeout.count() <= 0) {
    throw std::invalid_argument("stale_timeout must be positive");
  }
  CheckedState checked;
  checked.sample = std::move(sample);
  if (!checked.sample) {
    return checked;
  }
  const auto& raw = *checked.sample;
  checked.battery_valid = raw.battery_percent && std::isfinite(*raw.battery_percent) &&
                          *raw.battery_percent >= 0.0 && *raw.battery_percent <= 100.0;
  checked.velocity_valid = raw.body_velocity &&
      std::all_of(raw.body_velocity->begin(), raw.body_velocity->end(),
                  [](double value) { return std::isfinite(value); });
  checked.mode_valid = raw.mode.has_value();
  if (now < raw.received_at) {
    checked.reason = "SOURCE_TIME_IN_FUTURE";
    return checked;
  }
  const auto age = now - raw.received_at;
  checked.age_ms = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(age).count());
  if (age >= stale_timeout) {
    checked.reason = "STALE";
  } else if (!checked.battery_valid) {
    checked.reason = "INVALID_BATTERY";
  } else if (!checked.velocity_valid) {
    checked.reason = "INVALID_VELOCITY";
  } else if (!checked.mode_valid) {
    checked.reason = "MISSING_MODE";
  } else {
    checked.reason = "VALID";
    checked.valid = true;
  }
  return checked;
}
}  // namespace a2_platform
