#include "red_brick_detour_runtime.hpp"

#include <algorithm>
#include <atomic>

namespace red_event
{
namespace
{
std::atomic<std::uint64_t> s_pending_request{0};
std::atomic<std::uint64_t> s_runtime_state{0};

std::uint64_t Pack(std::uint32_t event_id, std::uint8_t high, std::uint8_t low)
{
    return (static_cast<std::uint64_t>(event_id) << 16) |
           (static_cast<std::uint64_t>(high) << 8) |
           static_cast<std::uint64_t>(low);
}
}

bool PublishBrickDetourRequest(const BrickDetourRequest &request)
{
    if (request.event_id == 0 || request.direction == AvoidDirection::kNone)
    {
        return false;
    }
    const std::uint64_t packed = Pack(
        request.event_id, 0, static_cast<std::uint8_t>(request.direction));
    std::uint64_t expected = 0;
    return s_pending_request.compare_exchange_strong(
        expected, packed, std::memory_order_release, std::memory_order_relaxed);
}

bool ConsumeBrickDetourRequest(BrickDetourRequest &request)
{
    const std::uint64_t packed = s_pending_request.exchange(0, std::memory_order_acq_rel);
    if (packed == 0)
    {
        request = {};
        return false;
    }
    request.event_id = static_cast<std::uint32_t>(packed >> 16);
    request.direction = static_cast<AvoidDirection>(packed & 0xffu);
    return request.event_id != 0 && request.direction != AvoidDirection::kNone;
}

void PublishBrickDetourState(const BrickDetourRuntimeSnapshot &snapshot)
{
    s_runtime_state.store(
        Pack(snapshot.event_id,
             static_cast<std::uint8_t>(snapshot.state),
             static_cast<std::uint8_t>(snapshot.direction)),
        std::memory_order_release);
}

BrickDetourRuntimeSnapshot LoadBrickDetourState()
{
    const std::uint64_t packed = s_runtime_state.load(std::memory_order_acquire);
    BrickDetourRuntimeSnapshot snapshot;
    snapshot.event_id = static_cast<std::uint32_t>(packed >> 16);
    snapshot.state = static_cast<BrickDetourState>((packed >> 8) & 0xffu);
    snapshot.direction = static_cast<AvoidDirection>(packed & 0xffu);
    return snapshot;
}

void ResetBrickDetourRuntime()
{
    s_pending_request.store(0, std::memory_order_release);
    s_runtime_state.store(0, std::memory_order_release);
}

float ApplyBrickDetourToImageError(float base_dir_error,
                                   const BrickDetourCommand &command)
{
    if (!command.active || command.ignore_image_steering)
    {
        return base_dir_error;
    }
    return base_dir_error - command.dir_offset;
}

float LimitBrickDetourDifferential(float speed_goal, float differential,
                                   bool detour_active)
{
    if (!detour_active)
    {
        return differential;
    }
    const float limit = std::max(0.0f, 0.5f * speed_goal);
    return std::max(-limit, std::min(differential, limit));
}
}
