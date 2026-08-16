#include <cmath>

#include <laser_tag_nav_planner/core/direction.h>

namespace laser_tag_nav_planner
{
namespace core
{

Vec2d directionStep(Direction direction)
{
  switch (direction)
  {
    case Direction::N: return {0.0, 1.0};
    case Direction::NE: return {1.0, 1.0};
    case Direction::E: return {1.0, 0.0};
    case Direction::SE: return {1.0, -1.0};
    case Direction::S: return {0.0, -1.0};
    case Direction::SW: return {-1.0, -1.0};
    case Direction::W: return {-1.0, 0.0};
    case Direction::NW: return {-1.0, 1.0};
  }
  return {0.0, 0.0};
}

double directionYaw(Direction direction)
{
  const Vec2d step = directionStep(direction);
  return std::atan2(step.y, step.x);
}

uint8_t directionBit(Direction direction)
{
  return static_cast<uint8_t>(1u << static_cast<int>(direction));
}

Direction opposite(Direction direction)
{
  return static_cast<Direction>((static_cast<int>(direction) + 4) % kDirectionCount);
}

double wrapToPi(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

}  // namespace core
}  // namespace laser_tag_nav_planner
