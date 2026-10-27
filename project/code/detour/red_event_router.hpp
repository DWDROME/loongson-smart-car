#pragma once

#include "red_event_types.hpp"

#include <cstdint>

namespace red_event
{
struct RedEventRouterConfig
{
    float white_ratio = 0.55f;
    std::uint8_t confirm_frames = 2;
    std::int16_t side_deadband = 5;
    bool brick_enable = false;
    bool picture_roi_enable = false;
};

class RedEventRouter
{
public:
    explicit RedEventRouter(const RedEventRouterConfig &config = {});

    void Configure(const RedEventRouterConfig &config);

    RedRouteResult Observe(const RedObservation &observation);
    RedRouteResult ObserveMissing();

    // Board becomes locked only after the single-frame NCNN result passes its confidence gate.
    RedRouteResult LockBoard(BoardRoute route);

    // Explicit event completion. This preserves the monotonic event id and starts a fresh confirmation.
    bool CompleteLockedEvent(std::uint32_t event_id);
    void Reset();

    const RedRouteResult &result() const { return result_; }
    const RedEventRouterConfig &config() const { return config_; }

private:
    RedRouteResult BuildLockedResult(bool new_event, bool run_board_ncnn) const;
    void ClearPending();
    void LockBrick(BrickSide side);

    RedRouteResult result_{};
    RedEventKind locked_kind_ = RedEventKind::kNone;
    BoardRoute locked_board_route_ = BoardRoute::kNone;
    BrickSide pending_brick_side_ = BrickSide::kUnknown;
    std::uint8_t pending_brick_count_ = 0;
    bool latest_board_candidate_ = false;
    std::uint32_t locked_event_id_ = 0;
    std::uint32_t next_event_id_ = 1;
    RedEventRouterConfig config_{};
};
}
