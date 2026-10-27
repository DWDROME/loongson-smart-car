#include "red_event_router.hpp"

#include <cmath>

namespace red_event
{
namespace
{
AvoidDirection AvoidForBrick(BrickSide side)
{
    if (side == BrickSide::kLeft)
    {
        return AvoidDirection::kRight;
    }
    if (side == BrickSide::kRight)
    {
        return AvoidDirection::kLeft;
    }
    return AvoidDirection::kNone;
}

}

RedEventRouter::RedEventRouter(const RedEventRouterConfig &config)
    : config_(config)
{
}

void RedEventRouter::Configure(const RedEventRouterConfig &config)
{
    config_ = config;
    Reset();
}

const char *ToString(RedEventKind kind)
{
    switch (kind)
    {
    case RedEventKind::kNone: return "none";
    case RedEventKind::kBoardCandidate: return "board-candidate";
    case RedEventKind::kBrickLeftCandidate: return "brick-left-candidate";
    case RedEventKind::kBrickRightCandidate: return "brick-right-candidate";
    case RedEventKind::kBoardLocked: return "board-locked";
    case RedEventKind::kBrickLeftLocked: return "brick-left-locked";
    case RedEventKind::kBrickRightLocked: return "brick-right-locked";
    }
    return "invalid";
}

const char *ToString(BoardRoute route)
{
    switch (route)
    {
    case BoardRoute::kNone: return "none";
    case BoardRoute::kRight: return "right";
    case BoardRoute::kStraight: return "straight";
    case BoardRoute::kLeft: return "left";
    }
    return "invalid";
}

const char *ToString(BrickSide side)
{
    switch (side)
    {
    case BrickSide::kUnknown: return "unknown";
    case BrickSide::kLeft: return "left";
    case BrickSide::kRight: return "right";
    }
    return "invalid";
}

const char *ToString(AvoidDirection direction)
{
    switch (direction)
    {
    case AvoidDirection::kNone: return "none";
    case AvoidDirection::kLeft: return "left";
    case AvoidDirection::kRight: return "right";
    }
    return "invalid";
}

RedRouteResult RedEventRouter::BuildLockedResult(bool new_event, bool run_board_ncnn) const
{
    RedRouteResult locked;
    locked.kind = locked_kind_;
    locked.board_route = locked_board_route_;
    locked.locked = true;
    locked.new_event = new_event;
    locked.run_board_ncnn = run_board_ncnn;
    locked.event_id = locked_event_id_;
    locked.confirm_count = (locked_kind_ == RedEventKind::kBrickLeftLocked ||
                            locked_kind_ == RedEventKind::kBrickRightLocked)
                               ? config_.confirm_frames
                               : 0;
    if (locked_kind_ == RedEventKind::kBrickLeftLocked)
    {
        locked.brick_side = BrickSide::kLeft;
    }
    else if (locked_kind_ == RedEventKind::kBrickRightLocked)
    {
        locked.brick_side = BrickSide::kRight;
    }
    locked.avoid_direction = AvoidForBrick(locked.brick_side);
    return locked;
}

void RedEventRouter::ClearPending()
{
    pending_brick_side_ = BrickSide::kUnknown;
    pending_brick_count_ = 0;
    latest_board_candidate_ = false;
}

void RedEventRouter::LockBrick(BrickSide side)
{
    locked_kind_ = (side == BrickSide::kLeft)
                       ? RedEventKind::kBrickLeftLocked
                       : RedEventKind::kBrickRightLocked;
    locked_board_route_ = BoardRoute::kNone;
    locked_event_id_ = next_event_id_++;
    result_ = BuildLockedResult(true, false);
    ClearPending();
}

RedRouteResult RedEventRouter::Observe(const RedObservation &observation)
{
    if (locked_kind_ != RedEventKind::kNone)
    {
        const bool same_board = locked_kind_ == RedEventKind::kBoardLocked &&
                                config_.picture_roi_enable &&
                                observation.valid &&
                                observation.white_ratio >= config_.white_ratio;
        result_ = BuildLockedResult(false, same_board);
        return result_;
    }

    if (!config_.brick_enable && !config_.picture_roi_enable)
    {
        ClearPending();
        result_ = {};
        return result_;
    }

    if (!observation.valid || !std::isfinite(observation.white_ratio))
    {
        return ObserveMissing();
    }

    if (observation.white_ratio >= config_.white_ratio)
    {
        ClearPending();
        latest_board_candidate_ = true;
        result_ = {};
        result_.kind = RedEventKind::kBoardCandidate;
        result_.run_board_ncnn = config_.picture_roi_enable;
        return result_;
    }

    latest_board_candidate_ = false;
    if (!config_.brick_enable)
    {
        ClearPending();
        latest_board_candidate_ = true;
        result_ = {};
        result_.kind = RedEventKind::kBoardCandidate;
        return result_;
    }
    if (observation.signed_side_error >= -config_.side_deadband &&
        observation.signed_side_error <= config_.side_deadband)
    {
        ClearPending();
        result_ = {};
        return result_;
    }

    const BrickSide side = observation.signed_side_error < 0
                               ? BrickSide::kLeft
                               : BrickSide::kRight;
    if (pending_brick_side_ != side)
    {
        pending_brick_side_ = side;
        pending_brick_count_ = 1;
    }
    else if (pending_brick_count_ < config_.confirm_frames)
    {
        ++pending_brick_count_;
    }

    if (pending_brick_count_ >= config_.confirm_frames)
    {
        LockBrick(side);
        return result_;
    }

    result_ = {};
    result_.kind = side == BrickSide::kLeft
                       ? RedEventKind::kBrickLeftCandidate
                       : RedEventKind::kBrickRightCandidate;
    result_.brick_side = side;
    result_.avoid_direction = AvoidForBrick(side);
    result_.confirm_count = pending_brick_count_;
    return result_;
}

RedRouteResult RedEventRouter::ObserveMissing()
{
    if (locked_kind_ != RedEventKind::kNone)
    {
        result_ = BuildLockedResult(false, false);
        return result_;
    }
    ClearPending();
    result_ = {};
    return result_;
}

RedRouteResult RedEventRouter::LockBoard(BoardRoute route)
{
    if (locked_kind_ != RedEventKind::kNone)
    {
        result_ = BuildLockedResult(false, locked_kind_ == RedEventKind::kBoardLocked);
        return result_;
    }
    if (!latest_board_candidate_ || route == BoardRoute::kNone)
    {
        return result_;
    }

    locked_kind_ = RedEventKind::kBoardLocked;
    locked_board_route_ = route;
    locked_event_id_ = next_event_id_++;
    result_ = BuildLockedResult(true, config_.picture_roi_enable);
    ClearPending();
    return result_;
}

bool RedEventRouter::CompleteLockedEvent(std::uint32_t event_id)
{
    if (locked_kind_ == RedEventKind::kNone || event_id == 0 ||
        event_id != locked_event_id_)
    {
        return false;
    }
    locked_kind_ = RedEventKind::kNone;
    locked_board_route_ = BoardRoute::kNone;
    locked_event_id_ = 0;
    ClearPending();
    result_ = {};
    return true;
}

void RedEventRouter::Reset()
{
    result_ = {};
    locked_kind_ = RedEventKind::kNone;
    locked_board_route_ = BoardRoute::kNone;
    pending_brick_side_ = BrickSide::kUnknown;
    pending_brick_count_ = 0;
    latest_board_candidate_ = false;
    locked_event_id_ = 0;
    next_event_id_ = 1;
}
}
