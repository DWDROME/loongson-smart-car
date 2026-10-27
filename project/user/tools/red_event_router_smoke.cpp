#include "detour/red_event_router.hpp"

#include <cstdio>

namespace
{
int failures = 0;

void Check(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

red_event::RedObservation Observation(float ratio, int side_error)
{
    red_event::RedObservation observation;
    observation.valid = true;
    observation.white_ratio = ratio;
    observation.signed_side_error = static_cast<std::int16_t>(side_error);
    return observation;
}
}

int main()
{
    using namespace red_event;

    RedEventRouterConfig default_config{};
    default_config.brick_enable = true;
    default_config.picture_roi_enable = true;
    RedEventRouter router(default_config);
    RedRouteResult result = router.Observe(Observation(0.55f, 20));
    Check(result.kind == RedEventKind::kBoardCandidate, "0.55 must route to board");
    Check(result.run_board_ncnn, "board candidate must request single-frame NCNN");
    result = router.LockBoard(BoardRoute::kStraight);
    Check(result.kind == RedEventKind::kBoardLocked && result.new_event &&
              result.board_route == BoardRoute::kStraight,
          "accepted board result must lock once as straight");
    const std::uint32_t board_event_id = result.event_id;
    result = router.Observe(Observation(0.30f, -20));
    Check(result.kind == RedEventKind::kBoardLocked && result.event_id == board_event_id,
          "brick cannot抢占 a locked board event");
    Check(!result.run_board_ncnn, "brick-looking frame under board lock must not run NCNN");
    Check(router.CompleteLockedEvent(board_event_id), "board event completion must be explicit");

    result = router.Observe(Observation(0.50f, -20));
    Check(result.kind == RedEventKind::kBrickLeftCandidate && result.confirm_count == 1,
          "0.50 must route below the 0.55 threshold as a brick candidate");
    result = router.Observe(Observation(0.50f, -20));
    Check(result.kind == RedEventKind::kBrickLeftLocked && result.new_event,
          "two left brick frames must lock");
    Check(result.avoid_direction == AvoidDirection::kRight,
          "left brick must request right avoidance");
    const std::uint32_t left_event_id = result.event_id;
    result = router.Observe(Observation(0.70f, 20));
    Check(result.kind == RedEventKind::kBrickLeftLocked &&
          result.event_id == left_event_id && !result.run_board_ncnn,
          "board cannot抢占 a locked brick event");
    Check(!router.CompleteLockedEvent(left_event_id + 1),
          "wrong event id must not release the lock");
    Check(router.CompleteLockedEvent(left_event_id), "left brick completion must succeed");

    router.Observe(Observation(0.30f, 20));
    result = router.Observe(Observation(0.30f, -20));
    Check(result.kind == RedEventKind::kBrickLeftCandidate && result.confirm_count == 1,
          "direction change must restart confirmation");
    result = router.Observe(Observation(0.60f, -20));
    Check(result.kind == RedEventKind::kBoardCandidate,
          "crossing threshold must clear brick confirmation");
    result = router.Observe(Observation(0.30f, -20));
    Check(result.confirm_count == 1, "board/brick jitter must not retain old count");
    result = router.ObserveMissing();
    Check(result.kind == RedEventKind::kNone, "missing candidate must clear pending count");

    result = router.Observe(Observation(0.20f, default_config.side_deadband));
    Check(result.kind == RedEventKind::kNone, "center deadband boundary must not guess a side");
    result = router.Observe(Observation(0.20f, 20));
    Check(result.kind == RedEventKind::kBrickRightCandidate, "right brick first frame candidate");
    result = router.Observe(Observation(0.20f, 20));
    Check(result.kind == RedEventKind::kBrickRightLocked,
          "two right brick frames must lock");
    Check(result.avoid_direction == AvoidDirection::kLeft,
          "right brick must request left avoidance");
    Check(result.event_id > left_event_id, "event id must be monotonic");

    RedEventRouterConfig custom_config;
    custom_config.white_ratio = 0.70f;
    custom_config.confirm_frames = 3;
    custom_config.side_deadband = 9;
    custom_config.brick_enable = true;
    RedEventRouter custom_router(custom_config);
    result = custom_router.Observe(Observation(0.69f, 10));
    Check(result.kind == RedEventKind::kBrickRightCandidate && result.confirm_count == 1,
          "custom white ratio and deadband must be used");
    result = custom_router.Observe(Observation(0.69f, 10));
    Check(result.kind == RedEventKind::kBrickRightCandidate && result.confirm_count == 2,
          "custom confirmation count must not lock early");
    result = custom_router.Observe(Observation(0.69f, 10));
    Check(result.kind == RedEventKind::kBrickRightLocked && result.confirm_count == 3,
          "custom confirmation count must lock at configured frame");

    RedEventRouterConfig disabled_config = default_config;
    disabled_config.brick_enable = false;
    RedEventRouter disabled_router(disabled_config);
    result = disabled_router.Observe(Observation(0.20f, -20));
    Check(result.kind == RedEventKind::kBoardCandidate && !result.run_board_ncnn,
          "disabled brick must pass straight without NCNN");
    result = disabled_router.LockBoard(BoardRoute::kStraight);
    Check(result.kind == RedEventKind::kBoardLocked &&
              result.board_route == BoardRoute::kStraight,
          "disabled brick passthrough must lock straight");

    RedEventRouterConfig roi_disabled_config = default_config;
    roi_disabled_config.picture_roi_enable = false;
    RedEventRouter roi_disabled_router(roi_disabled_config);
    result = roi_disabled_router.Observe(Observation(0.70f, 20));
    Check(result.kind == RedEventKind::kBoardCandidate && !result.run_board_ncnn,
          "disabled picture ROI must pass board straight without NCNN");

    RedEventRouterConfig all_disabled_config = default_config;
    all_disabled_config.brick_enable = false;
    all_disabled_config.picture_roi_enable = false;
    RedEventRouter all_disabled_router(all_disabled_config);
    result = all_disabled_router.Observe(Observation(0.70f, 20));
    Check(result.kind == RedEventKind::kNone && !result.run_board_ncnn,
          "disabling both red branches must ignore every red target");

    result = disabled_router.Observe(Observation(0.20f, -20));
    Check(result.kind == RedEventKind::kBoardLocked && result.confirm_count == 0,
          "disabled brick passthrough must not accumulate confirmation");
    disabled_router.CompleteLockedEvent(result.event_id);
    result = disabled_router.Observe(Observation(0.55f, 20));
    Check(result.kind == RedEventKind::kBoardCandidate && result.run_board_ncnn,
          "disabled brick must keep the board route");

    std::printf("red_event_router_smoke: %s\n", failures == 0 ? "passed" : "failed");
    return failures == 0 ? 0 : 1;
}
