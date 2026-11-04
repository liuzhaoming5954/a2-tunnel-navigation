#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <limits>
#include <optional>
#include <stdexcept>

#include "a2_platform/state_source.hpp"
#include "a2_platform/state_validation.hpp"

namespace a2_platform
{
namespace
{

using namespace std::chrono_literals;

RawState complete_sample(SteadyTime received_at)
{
  RawState sample;
  sample.sequence = 7;
  sample.received_at = received_at;
  sample.battery_percent = 82.0;
  sample.body_velocity = std::array<double, 3>{0.0, 0.0, 0.0};
  sample.mode = 0;
  return sample;
}

TEST(StateValidation, NoSampleIsUnavailableAndInvalid)
{
  const auto checked = check_state(std::nullopt, SteadyTime{} + 1s, 100ms);
  EXPECT_FALSE(checked.sample.has_value());
  EXPECT_FALSE(checked.valid);
  EXPECT_FALSE(checked.battery_valid);
  EXPECT_FALSE(checked.velocity_valid);
  EXPECT_FALSE(checked.mode_valid);
  EXPECT_EQ(checked.reason, "NO_DATA");
}

TEST(StateValidation, CompleteFreshSamplePreservesIdentityAndValues)
{
  const auto received = SteadyTime{} + 1s;
  const auto checked = check_state(complete_sample(received), received + 25ms, 100ms);
  ASSERT_TRUE(checked.sample.has_value());
  EXPECT_TRUE(checked.valid);
  EXPECT_TRUE(checked.battery_valid);
  EXPECT_TRUE(checked.velocity_valid);
  EXPECT_TRUE(checked.mode_valid);
  EXPECT_EQ(checked.age_ms, 25u);
  EXPECT_EQ(checked.reason, "VALID");
  EXPECT_EQ(checked.sample->sequence, 7u);
  EXPECT_EQ(checked.sample->received_at, received);
  EXPECT_DOUBLE_EQ(*checked.sample->battery_percent, 82.0);
}

TEST(StateValidation, TimeoutBoundaryIsInclusive)
{
  const auto received = SteadyTime{} + 1s;
  const auto sample = complete_sample(received);
  EXPECT_TRUE(check_state(sample, received + 99ms, 100ms).valid);
  const auto expired = check_state(sample, received + 100ms, 100ms);
  EXPECT_FALSE(expired.valid);
  EXPECT_EQ(expired.reason, "STALE");
  EXPECT_EQ(expired.age_ms, 100u);
}

TEST(StateValidation, FutureReceptionTimeCannotAppearFresh)
{
  const auto now = SteadyTime{} + 1s;
  const auto checked = check_state(complete_sample(now + 1ms), now, 100ms);
  EXPECT_FALSE(checked.valid);
  EXPECT_EQ(checked.reason, "SOURCE_TIME_IN_FUTURE");
}

TEST(StateValidation, MissingAndNonfiniteOrOutOfRangeBatteryIsInvalid)
{
  const auto now = SteadyTime{} + 1s;
  const std::array<std::optional<double>, 6> invalid_values{
    std::nullopt,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -0.1,
    100.1};
  for (std::size_t index = 0; index < invalid_values.size(); ++index) {
    SCOPED_TRACE(index);
    auto sample = complete_sample(now);
    sample.battery_percent = invalid_values[index];
    const auto checked = check_state(sample, now, 100ms);
    EXPECT_FALSE(checked.valid);
    EXPECT_FALSE(checked.battery_valid);
    EXPECT_TRUE(checked.velocity_valid);
    EXPECT_EQ(checked.reason, "INVALID_BATTERY");
  }
}

TEST(StateValidation, BatteryRangeEndpointsAreValid)
{
  const auto now = SteadyTime{} + 1s;
  for (const auto value : {0.0, 100.0}) {
    SCOPED_TRACE(value);
    auto sample = complete_sample(now);
    sample.battery_percent = value;
    const auto checked = check_state(sample, now, 100ms);
    EXPECT_TRUE(checked.battery_valid);
    EXPECT_TRUE(checked.valid);
  }
}

TEST(StateValidation, MissingVelocityIsInvalid)
{
  const auto now = SteadyTime{} + 1s;
  auto sample = complete_sample(now);
  sample.body_velocity.reset();
  const auto checked = check_state(sample, now, 100ms);
  EXPECT_FALSE(checked.valid);
  EXPECT_FALSE(checked.velocity_valid);
  EXPECT_EQ(checked.reason, "INVALID_VELOCITY");
}

TEST(StateValidation, EveryVelocityComponentMustBeFinite)
{
  const auto now = SteadyTime{} + 1s;
  for (std::size_t component = 0; component < 3; ++component) {
    for (const auto value : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
    {
      SCOPED_TRACE(component);
      auto sample = complete_sample(now);
      (*sample.body_velocity)[component] = value;
      const auto checked = check_state(sample, now, 100ms);
      EXPECT_FALSE(checked.valid);
      EXPECT_FALSE(checked.velocity_valid);
      EXPECT_TRUE(checked.battery_valid);
      EXPECT_EQ(checked.reason, "INVALID_VELOCITY");
    }
  }
}

TEST(StateValidation, MissingModeIsNotReportedAsModeZero)
{
  const auto now = SteadyTime{} + 1s;
  auto sample = complete_sample(now);
  sample.mode.reset();
  const auto checked = check_state(sample, now, 100ms);
  EXPECT_FALSE(checked.valid);
  EXPECT_FALSE(checked.mode_valid);
  EXPECT_EQ(checked.reason, "MISSING_MODE");
  EXPECT_FALSE(checked.sample->mode.has_value());
}

TEST(StateValidation, RepeatedChecksAgeSampleWithoutRefreshingIt)
{
  const auto received = SteadyTime{} + 1s;
  const auto sample = complete_sample(received);
  const auto first = check_state(sample, received + 10ms, 100ms);
  const auto last = check_state(sample, received + 250ms, 100ms);
  ASSERT_TRUE(last.sample.has_value());
  EXPECT_TRUE(first.valid);
  EXPECT_FALSE(last.valid);
  EXPECT_EQ(last.age_ms, 250u);
  EXPECT_EQ(last.reason, "STALE");
  EXPECT_TRUE(last.battery_valid);
  EXPECT_TRUE(last.velocity_valid);
  EXPECT_TRUE(last.mode_valid);
  EXPECT_EQ(last.sample->sequence, first.sample->sequence);
  EXPECT_EQ(last.sample->received_at, received);
}

TEST(StateValidation, NonpositiveTimeoutIsRejected)
{
  const auto now = SteadyTime{} + 1s;
  EXPECT_THROW(check_state(complete_sample(now), now, 0ms), std::invalid_argument);
  EXPECT_THROW(check_state(std::nullopt, now, -1ms), std::invalid_argument);
}

TEST(MockStateSource, FreshCallsProduceNewSamplesUsingInjectedClock)
{
  auto now = SteadyTime{} + 1s;
  MockStateSource source([&now]() {return now;});
  const auto first = source.latest();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->received_at, now);
  EXPECT_DOUBLE_EQ(*first->battery_percent, 82.0);
  EXPECT_EQ(*first->body_velocity, (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_EQ(*first->mode, 0u);
  now += 20ms;
  const auto second = source.latest();
  ASSERT_TRUE(second.has_value());
  EXPECT_GT(second->sequence, first->sequence);
  EXPECT_EQ(second->received_at, now);
}

TEST(MockStateSource, DropAtBoundaryFreezesLastSampleAndEventuallyBecomesStale)
{
  auto now = SteadyTime{} + 1s;
  MockStateSource source([&now]() {return now;}, 50ms);
  ASSERT_TRUE(source.latest().has_value());
  now += 49ms;
  const auto last_good = source.latest();
  ASSERT_TRUE(last_good.has_value());
  now += 1ms;
  const auto at_drop = source.latest();
  ASSERT_TRUE(at_drop.has_value());
  EXPECT_EQ(at_drop->sequence, last_good->sequence);
  EXPECT_EQ(at_drop->received_at, last_good->received_at);
  now += 500ms;
  const auto frozen = source.latest();
  ASSERT_TRUE(frozen.has_value());
  EXPECT_EQ(frozen->sequence, last_good->sequence);
  EXPECT_EQ(frozen->received_at, last_good->received_at);
  const auto checked = check_state(frozen, now, 100ms);
  EXPECT_FALSE(checked.valid);
  EXPECT_EQ(checked.reason, "STALE");
  EXPECT_EQ(checked.age_ms, 501u);
}

TEST(MockStateSource, ImmediateDropHasNoSyntheticSampleAndNegativeDropIsRejected)
{
  auto now = SteadyTime{} + 1s;
  MockStateSource source([&now]() {return now;}, 0ms);
  EXPECT_FALSE(source.latest().has_value());
  now += 10s;
  EXPECT_FALSE(source.latest().has_value());
  EXPECT_THROW((MockStateSource{[&now]() {return now;}, -1ms}), std::invalid_argument);
}

}  // namespace
}  // namespace a2_platform
