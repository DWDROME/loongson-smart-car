#pragma once

#include "red_event_types.hpp"

#include <cstdint>

namespace red_event
{
enum class BrickDetourState : std::uint8_t
{
    kIdle = 0,
    kSlowdown,
    kOut,
    kPast,
    kBack,
    kRecover,
    kDone,
};

struct BrickDetourCommand
{
    bool active = false;
    bool ignore_image_steering = false;
    bool speed_override_valid = false;
    float speed_goal = 0.0f;
    float dir_offset = 0.0f;
};

struct RedBrickDetourConfig
{
    std::uint32_t slowdown_ms = 333;
    std::uint32_t out_ms = 1167;
    std::uint32_t past_ms = 667;
    std::uint32_t back_ms = 833;
    std::uint32_t recover_ms = 500;
    float slow_speed = 100.0f;
    float past_speed = 220.0f;
    float offset_out = 8.0f;
    float offset_back = 10.0f;
};

class RedBrickDetour
{
public:
    explicit RedBrickDetour(const RedBrickDetourConfig &config = {});

    void Configure(const RedBrickDetourConfig &config);

    bool Start(AvoidDirection direction, std::uint64_t now_ms);
    BrickDetourCommand Update(std::uint64_t now_ms);
    void Cancel();

    BrickDetourState state() const { return state_; }
    AvoidDirection direction() const { return direction_; }
    bool active() const;
    bool done() const { return state_ == BrickDetourState::kDone; }
    const BrickDetourCommand &command() const { return command_; }
    const RedBrickDetourConfig &config() const { return config_; }
    static BrickDetourCommand CommandFor(BrickDetourState state,
                                         AvoidDirection direction,
                                         const RedBrickDetourConfig &config);

private:
    std::uint32_t StateDurationMs() const;
    void AdvanceOneState();
    void RefreshCommand();

    BrickDetourState state_ = BrickDetourState::kIdle;
    AvoidDirection direction_ = AvoidDirection::kNone;
    std::uint64_t state_entry_ms_ = 0;
    BrickDetourCommand command_{};
    RedBrickDetourConfig config_{};
};

const char *ToString(BrickDetourState state);
}
