#pragma once

#include <cstdint>

namespace red_event
{
enum class RedEventKind : std::uint8_t
{
    kNone = 0,
    kBoardCandidate,
    kBrickLeftCandidate,
    kBrickRightCandidate,
    kBoardLocked,
    kBrickLeftLocked,
    kBrickRightLocked,
};

enum class BoardRoute : std::uint8_t
{
    kNone = 0,
    kRight,
    kStraight,
    kLeft,
};

// Physical brick position relative to the lane center.
enum class BrickSide : std::uint8_t
{
    kUnknown = 0,
    kLeft,
    kRight,
};

// Requested vehicle avoidance direction. It is opposite to BrickSide.
enum class AvoidDirection : std::uint8_t
{
    kNone = 0,
    kLeft,
    kRight,
};

struct RedObservation
{
    bool valid = false;
    float white_ratio = 0.0f;
    std::int16_t signed_side_error = 0;
};

struct RedRouteResult
{
    RedEventKind kind = RedEventKind::kNone;
    BoardRoute board_route = BoardRoute::kNone;
    BrickSide brick_side = BrickSide::kUnknown;
    AvoidDirection avoid_direction = AvoidDirection::kNone;
    std::uint8_t confirm_count = 0;
    bool locked = false;
    bool new_event = false;
    bool run_board_ncnn = false;
    std::uint32_t event_id = 0;
};

const char *ToString(RedEventKind kind);
const char *ToString(BoardRoute route);
const char *ToString(BrickSide side);
const char *ToString(AvoidDirection direction);
}
