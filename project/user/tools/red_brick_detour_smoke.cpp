#include "detour/red_brick_detour.hpp"
#include "detour/red_brick_detour_runtime.hpp"

#include <cmath>
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

bool Near(float lhs, float rhs)
{
    return std::fabs(lhs - rhs) < 0.001f;
}

void CheckNeutral(const red_event::BrickDetourCommand &command, const char *where)
{
    Check(!command.active, where);
    Check(!command.ignore_image_steering, "neutral command must not ignore image steering");
    Check(!command.speed_override_valid, "neutral command must not override speed");
    Check(Near(command.speed_goal, 0.0f), "neutral speed must be zero-valued");
    Check(Near(command.dir_offset, 0.0f), "neutral offset must be zero");
}
}

int main()
{
    using namespace red_event;

    const RedBrickDetourConfig default_config{};
    RedBrickDetour detour(default_config);
    CheckNeutral(detour.command(), "constructor must be neutral");
    Check(!detour.Start(AvoidDirection::kNone, 1000), "none direction must not start");
    Check(detour.Start(AvoidDirection::kLeft, 1000), "left avoidance must start");
    Check(detour.state() == BrickDetourState::kSlowdown, "start enters slowdown");
    Check(detour.command().speed_override_valid &&
          Near(detour.command().speed_goal, default_config.slow_speed),
          "slowdown must request speed 100");
    Check(!detour.Start(AvoidDirection::kRight, 1001), "active detour direction is immutable");

    BrickDetourCommand command = detour.Update(1000 + default_config.slowdown_ms - 1);
    Check(detour.state() == BrickDetourState::kSlowdown, "slowdown boundary minus one");
    command = detour.Update(1000 + default_config.slowdown_ms);
    Check(detour.state() == BrickDetourState::kOut, "slowdown boundary enters out");
    Check(Near(command.dir_offset, -default_config.offset_out),
          "left avoidance out offset must be negative");

    const std::uint64_t out_end =
        1000 + default_config.slowdown_ms + default_config.out_ms;
    command = detour.Update(out_end);
    Check(detour.state() == BrickDetourState::kPast, "out boundary enters past");
    Check(command.ignore_image_steering && command.speed_override_valid &&
          Near(command.speed_goal, default_config.past_speed),
          "past must ignore image and request speed 220");

    const std::uint64_t past_end = out_end + default_config.past_ms;
    command = detour.Update(past_end);
    Check(detour.state() == BrickDetourState::kBack, "past boundary enters back");
    Check(Near(command.dir_offset, default_config.offset_back),
          "left avoidance back offset must reverse sign");

    const std::uint64_t back_end = past_end + default_config.back_ms;
    command = detour.Update(back_end);
    Check(detour.state() == BrickDetourState::kRecover, "back boundary enters recover");
    Check(command.active && !command.ignore_image_steering &&
          !command.speed_override_valid && Near(command.dir_offset, 0.0f),
          "recover must use normal image steering with no overrides");

    const std::uint64_t recover_end = back_end + default_config.recover_ms;
    command = detour.Update(recover_end);
    Check(detour.state() == BrickDetourState::kDone, "recover boundary enters done");
    CheckNeutral(command, "done must be neutral");
    command = detour.Update(recover_end + 5000);
    Check(detour.state() == BrickDetourState::kDone, "done must hold");
    CheckNeutral(command, "held done must remain neutral");

    Check(detour.Start(AvoidDirection::kRight, recover_end + 6000),
          "explicit start after done must begin a new detour");
    detour.Update(recover_end + 6000 + default_config.slowdown_ms);
    Check(Near(detour.command().dir_offset, default_config.offset_out),
          "right avoidance out offset must be positive");
    detour.Cancel();
    Check(detour.state() == BrickDetourState::kIdle, "cancel must return idle");
    CheckNeutral(detour.command(), "cancel must clear every output");

    Check(detour.Start(AvoidDirection::kRight, 20000), "jump test must start");
    detour.Update(30000);
    Check(detour.state() == BrickDetourState::kOut,
          "one Update call may advance at most one state even after a long delay");

    ResetBrickDetourRuntime();
    BrickDetourRequest request;
    request.event_id = 17;
    request.direction = AvoidDirection::kLeft;
    Check(PublishBrickDetourRequest(request), "first runtime request must publish");
    Check(!PublishBrickDetourRequest(request), "pending runtime request must not be overwritten");
    BrickDetourRequest consumed;
    Check(ConsumeBrickDetourRequest(consumed) && consumed.event_id == 17 &&
          consumed.direction == AvoidDirection::kLeft,
          "control thread must consume the exact image request once");
    Check(!ConsumeBrickDetourRequest(consumed), "runtime request must be single-consumer");

    BrickDetourRuntimeSnapshot snapshot;
    snapshot.event_id = 17;
    snapshot.state = BrickDetourState::kOut;
    snapshot.direction = AvoidDirection::kLeft;
    PublishBrickDetourState(snapshot);
    const BrickDetourRuntimeSnapshot loaded = LoadBrickDetourState();
    Check(loaded.event_id == 17 && loaded.state == BrickDetourState::kOut &&
          loaded.direction == AvoidDirection::kLeft,
          "runtime state snapshot must round-trip atomically");
    const BrickDetourCommand left_out =
        RedBrickDetour::CommandFor(loaded.state, loaded.direction, default_config);
    Check(Near(left_out.dir_offset, -default_config.offset_out),
          "left OUT command must use the current tuned offset");
    Check(Near(ApplyBrickDetourToImageError(0.0f, left_out),
               default_config.offset_out),
          "current Dir_err polarity must invert provider offset for a real left turn");
    const BrickDetourCommand past =
        RedBrickDetour::CommandFor(BrickDetourState::kPast, AvoidDirection::kLeft,
                                   default_config);
    Check(Near(ApplyBrickDetourToImageError(7.0f, past), 7.0f),
          "PAST image-ignore command must not inject a stale pixel offset");
    Check(Near(LimitBrickDetourDifferential(100.0f, 80.0f, true), 50.0f) &&
          Near(LimitBrickDetourDifferential(100.0f, -80.0f, true), -50.0f),
          "active detour must cap differential at speed/2 to prevent wheel reversal");
    Check(Near(LimitBrickDetourDifferential(100.0f, 80.0f, false), 80.0f),
          "inactive detour must not change the existing differential command");
    CheckNeutral(RedBrickDetour::CommandFor(BrickDetourState::kOut,
                                             AvoidDirection::kNone,
                                             default_config),
                 "missing direction must not create an active command");

    RedBrickDetourConfig custom_config;
    custom_config.slowdown_ms = 11;
    custom_config.out_ms = 12;
    custom_config.past_ms = 13;
    custom_config.back_ms = 14;
    custom_config.recover_ms = 15;
    custom_config.slow_speed = 123.0f;
    custom_config.past_speed = 234.0f;
    custom_config.offset_out = 7.0f;
    custom_config.offset_back = 9.0f;
    RedBrickDetour custom(custom_config);
    Check(custom.Start(AvoidDirection::kRight, 50000), "custom detour must start");
    Check(Near(custom.command().speed_goal, 123.0f), "custom slow speed must apply");
    command = custom.Update(50011);
    Check(custom.state() == BrickDetourState::kOut && Near(command.dir_offset, 7.0f),
          "custom slowdown duration and OUT offset must apply");
    command = custom.Update(50023);
    Check(custom.state() == BrickDetourState::kPast &&
          Near(command.speed_goal, 234.0f),
          "custom OUT duration and PAST speed must apply");
    command = custom.Update(50036);
    Check(custom.state() == BrickDetourState::kBack &&
          Near(command.dir_offset, -9.0f),
          "custom PAST duration and BACK offset must apply");
    command = custom.Update(50050);
    Check(custom.state() == BrickDetourState::kRecover,
          "custom BACK duration must apply");
    command = custom.Update(50065);
    Check(custom.state() == BrickDetourState::kDone,
          "custom RECOVER duration must apply");
    CheckNeutral(command, "custom done must be neutral");

    std::printf("red_brick_detour_smoke: %s\n", failures == 0 ? "passed" : "failed");
    return failures == 0 ? 0 : 1;
}
