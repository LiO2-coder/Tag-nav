#ifndef TAG_NAV_PLANNER_CORE_DIRECTION_H
#define TAG_NAV_PLANNER_CORE_DIRECTION_H

#include <tag_nav_planner/core/types.h>

namespace tag_nav_planner
{
namespace core
{

constexpr int kDirectionCount = 8;

// Unit grid step (dx, dy) for each direction, in a +x = east, +y = north frame.
Vec2d directionStep(Direction direction);

// Heading angle (atan2(dy, dx)) for a direction, in radians, in [-pi, pi].
double directionYaw(Direction direction);

// Connectivity mask bit for a direction: 1 << static_cast<int>(direction).
uint8_t directionBit(Direction direction);

// Opposite direction: (direction + 4) % 8.
Direction opposite(Direction direction);

// Wraps an angle into [-pi, pi].
double wrapToPi(double angle);

}  // namespace core
}  // namespace tag_nav_planner

#endif
