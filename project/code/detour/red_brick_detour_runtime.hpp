#pragma once

#include "red_brick_detour.hpp"

#include <cstdint>

namespace red_event
{
struct BrickDetourRequest
{
    std::uint32_t event_id = 0;
    AvoidDirection direction = AvoidDirection::kNone;
};

struct BrickDetourRuntimeSnapshot
{
    std::uint32_t event_id = 0;
    BrickDetourState state = BrickDetourState::kIdle;
    AvoidDirection direction = AvoidDirection::kNone;
};

// Image thread -> 5 ms control thread. A pending request is never overwritten.
bool PublishBrickDetourRequest(const BrickDetourRequest &request);
bool ConsumeBrickDetourRequest(BrickDetourRequest &request);

// 5 ms control thread -> image thread/read-only diagnostics.
void PublishBrickDetourState(const BrickDetourRuntimeSnapshot &snapshot);
BrickDetourRuntimeSnapshot LoadBrickDetourState();
void ResetBrickDetourRuntime();

// Provider offset uses +right/-left. Current Dir_err control polarity is opposite,
// so the provider command must be subtracted at the existing image-error entry.
float ApplyBrickDetourToImageError(float base_dir_error,
                                   const BrickDetourCommand &command);
float LimitBrickDetourDifferential(float speed_goal, float differential,
                                   bool detour_active);
}
