#include <franka/robot.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

using Point = std::array<double, 3>;
using Pose = std::array<double, 16>;

constexpr std::size_t translation_index = 12;
constexpr double peak_speed = 0.05;  // m/s

void printUsage(std::ostream& out, const char* program) {
  out << "Usage: " << program << " ax ay az bx by bz\n"
      << "Enter absolute end-effector coordinates in the robot base frame, in meters.\n"
      << "The arm moves from its starting pose to A, then B, with fixed orientation.\n";
}

double parseCoordinate(const char* argument) {
  const std::string value(argument);
  std::size_t parsed = 0;
  const double coordinate = std::stod(value, &parsed);
  if (parsed != value.size() || !std::isfinite(coordinate)) {
    throw std::invalid_argument("Coordinates must be finite numbers");
  }
  return coordinate;
}

double segmentDuration(const Point& start, const Point& end) {
  const double distance =
      std::hypot(end[0] - start[0], end[1] - start[1], end[2] - start[2]);
  // A quintic blend reaches its peak normalized speed of 1.875 at the midpoint.
  return std::max(1.0, (1.875 / peak_speed) * distance);
}

void interpolate(Pose& pose, const Point& start, const Point& end, double fraction) {
  // Zero velocity and acceleration at both ends of the straight-line segment.
  const double blend = fraction * fraction * fraction *
                       (10.0 + fraction * (-15.0 + 6.0 * fraction));
  for (std::size_t axis = 0; axis < start.size(); ++axis) {
    pose[translation_index + axis] = start[axis] + blend * (end[axis] - start[axis]);
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    printUsage(std::cout, argv[0]);
    return 0;
  }
  if (argc != 7) {
    printUsage(std::cerr, argv[0]);
    return 2;
  }

  Point a{};
  Point b{};
  try {
    for (std::size_t axis = 0; axis < a.size(); ++axis) {
      a[axis] = parseCoordinate(argv[1 + axis]);
      b[axis] = parseCoordinate(argv[4 + axis]);
    }
  } catch (const std::exception&) {
    std::cerr << "Coordinates must be finite numbers in meters.\n";
    return 2;
  }

  const Point base_origin{};
  if (!std::isfinite(segmentDuration(base_origin, a) + segmentDuration(a, b))) {
    std::cerr << "Coordinates are too large to compute a trajectory.\n";
    return 2;
  }

  try {
    franka::Robot robot(
        "172.16.0.2",
        franka::RealtimeConfig::kIgnore
    );
    Pose pose{};
    Point start{};
    double elapsed = 0.0;
    double time_to_a = 0.0;
    const double time_to_b = segmentDuration(a, b);

    robot.control([&](const franka::RobotState& state, franka::Duration period)
                      -> franka::CartesianPose {
      if (time_to_a == 0.0) {
        pose = state.O_T_EE_c;
        std::copy_n(pose.begin() + translation_index, start.size(), start.begin());
        time_to_a = segmentDuration(start, a);
        return franka::CartesianPose(pose);
      }

      elapsed += period.toSec();
      if (elapsed <= time_to_a) {
        interpolate(pose, start, a, elapsed / time_to_a);
      } else {
        interpolate(pose, a, b, std::min(1.0, (elapsed - time_to_a) / time_to_b));
      }

      const franka::CartesianPose command(pose);
      constexpr double final_hold_seconds = 0.25;
      if (elapsed >= time_to_a + time_to_b + final_hold_seconds) {
        return franka::MotionFinished(command);
      }
      return command;
    });
    std::cout << "Cartesian trajectory completed.\n";
  } catch (const std::exception& error) {
    std::cerr << "Cartesian motion failed: " << error.what() << '\n';
    return 1;
  }
}
