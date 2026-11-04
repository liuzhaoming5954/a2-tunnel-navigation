// Optional SDK2 receive-only transport. Reference SDK revision:
// 8e1b927586712edd6a4c29d50dd52dff1e163439.
// This translation unit is excluded from the default mock build.
#include "a2_platform/state_source.hpp"

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <algorithm>
#include <cctype>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace a2_platform {
namespace {

// The official A2 state example explicitly uses this IDL and this topic.
// The shared go2 header is intentional, not an inferred model substitution.
using VendorState = unitree_go::msg::dds_::SportModeState_;
constexpr const char* kSportStateTopic = "rt/lf/sportmodestate";

// ChannelFactory is a process singleton. Do not silently change its domain or
// NIC when a second reader is constructed in the same process.
std::mutex factory_mutex;
std::optional<UnitreeConfig> initialized_factory;

void initialize_factory(const UnitreeConfig& config) {
  if (config.network_interface.empty() ||
      std::all_of(config.network_interface.begin(), config.network_interface.end(),
                  [](unsigned char value) { return std::isspace(value) != 0; })) {
    throw std::invalid_argument("Unitree readonly source requires an explicit network interface");
  }
  if (config.domain_id < 0) {
    throw std::invalid_argument("Unitree domain_id must be nonnegative");
  }
  std::lock_guard<std::mutex> lock(factory_mutex);
  if (initialized_factory) {
    if (initialized_factory->network_interface != config.network_interface ||
        initialized_factory->domain_id != config.domain_id) {
      throw std::invalid_argument("Unitree ChannelFactory is already initialized with another configuration");
    }
    return;
  }
  unitree::robot::ChannelFactory::Instance()->Init(
      config.domain_id, config.network_interface);
  initialized_factory = config;
}

struct StateCache {
  std::mutex mutex;
  bool accepting = true;
  uint64_t sequence = 0;
  std::optional<RawState> sample;
};

class UnitreeStateSource final : public StateSource {
 public:
  explicit UnitreeStateSource(const UnitreeConfig& config)
      : cache_(std::make_shared<StateCache>()) {
    initialize_factory(config);
    subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<VendorState>>(
        kSportStateTopic);
    // Capture shared state, never `this`: a queued callback cannot access a
    // destroyed source. CloseChannel releases this reader, not the global DDS
    // factory which may still be used by another receive-only source.
    const auto cache = cache_;
    subscriber_->InitChannel(
        [cache](const void* message) {
          if (message == nullptr) {
            return;
          }
          const auto received_at = std::chrono::steady_clock::now();
          const auto& state = *static_cast<const VendorState*>(message);
          std::lock_guard<std::mutex> lock(cache->mutex);
          if (!cache->accepting || cache->sequence == std::numeric_limits<uint64_t>::max()) {
            return;
          }
          RawState sample;
          sample.sequence = ++cache->sequence;
          sample.received_at = received_at;
          sample.mode = static_cast<uint32_t>(state.mode());
          // SportModeState_ has no battery field. Its velocity reference frame
          // is not documented by the inspected A2 example/IDL, so do not label
          // that vector as base_link body velocity.
          sample.battery_percent = std::nullopt;
          sample.body_velocity = std::nullopt;
          cache->sample = sample;
        },
        1);
  }

  ~UnitreeStateSource() override {
    {
      std::lock_guard<std::mutex> lock(cache_->mutex);
      cache_->accepting = false;
    }
    if (subscriber_) {
      subscriber_->CloseChannel();
    }
  }

  std::optional<RawState> latest() override {
    std::lock_guard<std::mutex> lock(cache_->mutex);
    return cache_->sample;
  }

 private:
  std::shared_ptr<StateCache> cache_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<VendorState>> subscriber_;
};

}  // namespace

std::unique_ptr<StateSource> make_unitree_state_source(const UnitreeConfig& config) {
  return std::make_unique<UnitreeStateSource>(config);
}

}  // namespace a2_platform
