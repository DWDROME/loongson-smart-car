#include "red_brick_detour.hpp"

namespace red_event
{
RedBrickDetour::RedBrickDetour(const RedBrickDetourConfig &config)
    : config_(config)
{
}

void RedBrickDetour::Configure(const RedBrickDetourConfig &config)
{
    config_ = config;
    Cancel();
}

const char *ToString(BrickDetourState state)
{
    switch (state)
    {
    case BrickDetourState::kIdle: return "idle";
    case BrickDetourState::kSlowdown: return "slowdown";
    case BrickDetourState::kOut: return "out";
    case BrickDetourState::kPast: return "past";
    case BrickDetourState::kBack: return "back";
    case BrickDetourState::kRecover: return "recover";
    case BrickDetourState::kDone: return "done";
    }
    return "invalid";
}

bool RedBrickDetour::Start(AvoidDirection direction, std::uint64_t now_ms)
{
    if (direction == AvoidDirection::kNone || active())
    {
        return false;
    }
    direction_ = direction;
    state_ = BrickDetourState::kSlowdown;
    state_entry_ms_ = now_ms;
    RefreshCommand();
    return true;
}

BrickDetourCommand RedBrickDetour::Update(std::uint64_t now_ms)
{
    const std::uint32_t duration_ms = StateDurationMs();
    if (duration_ms > 0 && now_ms >= state_entry_ms_ &&
        now_ms - state_entry_ms_ >= duration_ms)
    {
        state_entry_ms_ += duration_ms;
        AdvanceOneState();
    }
    RefreshCommand();
    return command_;
}

void RedBrickDetour::Cancel()
{
    state_ = BrickDetourState::kIdle;
    direction_ = AvoidDirection::kNone;
    state_entry_ms_ = 0;
    command_ = {};
}

bool RedBrickDetour::active() const
{
    return state_ != BrickDetourState::kIdle && state_ != BrickDetourState::kDone;
}

std::uint32_t RedBrickDetour::StateDurationMs() const
{
    switch (state_)
    {
    case BrickDetourState::kSlowdown: return config_.slowdown_ms;
    case BrickDetourState::kOut: return config_.out_ms;
    case BrickDetourState::kPast: return config_.past_ms;
    case BrickDetourState::kBack: return config_.back_ms;
    case BrickDetourState::kRecover: return config_.recover_ms;
    case BrickDetourState::kIdle:
    case BrickDetourState::kDone:
        return 0;
    }
    return 0;
}

void RedBrickDetour::AdvanceOneState()
{
    switch (state_)
    {
    case BrickDetourState::kSlowdown: state_ = BrickDetourState::kOut; break;
    case BrickDetourState::kOut: state_ = BrickDetourState::kPast; break;
    case BrickDetourState::kPast: state_ = BrickDetourState::kBack; break;
    case BrickDetourState::kBack: state_ = BrickDetourState::kRecover; break;
    case BrickDetourState::kRecover:
        state_ = BrickDetourState::kDone;
        direction_ = AvoidDirection::kNone;
        break;
    case BrickDetourState::kIdle:
    case BrickDetourState::kDone:
        break;
    }
}

void RedBrickDetour::RefreshCommand()
{
    command_ = CommandFor(state_, direction_, config_);
}

BrickDetourCommand RedBrickDetour::CommandFor(BrickDetourState state,
                                               AvoidDirection direction,
                                               const RedBrickDetourConfig &config)
{
    BrickDetourCommand command;
    if (direction == AvoidDirection::kNone &&
        state != BrickDetourState::kIdle && state != BrickDetourState::kDone)
    {
        return command;
    }
    const float direction_sign = direction == AvoidDirection::kLeft ? -1.0f : 1.0f;
    switch (state)
    {
    case BrickDetourState::kSlowdown:
        command.active = true;
        command.speed_override_valid = true;
        command.speed_goal = config.slow_speed;
        break;
    case BrickDetourState::kOut:
        command.active = true;
        command.dir_offset = direction_sign * config.offset_out;
        break;
    case BrickDetourState::kPast:
        command.active = true;
        command.ignore_image_steering = true;
        command.speed_override_valid = true;
        command.speed_goal = config.past_speed;
        break;
    case BrickDetourState::kBack:
        command.active = true;
        command.dir_offset = -direction_sign * config.offset_back;
        break;
    case BrickDetourState::kRecover:
        command.active = true;
        break;
    case BrickDetourState::kIdle:
    case BrickDetourState::kDone:
        break;
    }
    return command;
}
}
