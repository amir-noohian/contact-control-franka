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

using Joints = std::array<double, 7>;

// Franka Research 3 joint position limits, in radians.
constexpr Joints lower{-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508};
constexpr Joints upper{2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
constexpr double peak_speed = 0.15;  // rad/s per joint

void printUsage(std::ostream& out, const char* program) {
  out << "Usage: " << program << " Q1 Q2 Q3 Q4 Q5 Q6 Q7\n"
      << "Enter seven absolute FR3 joint angles in radians.\n";
}

Joints parseDestination(char** args) {
  Joints waypoint{};
  for (std::size_t joint = 0; joint < waypoint.size(); ++joint) {
    const std::string input(args[joint]);
    std::size_t parsed = 0;
    const double angle = std::stod(input, &parsed);
    if (parsed != input.size() || !std::isfinite(angle) ||
        angle <= lower[joint] || angle >= upper[joint]) {
      throw std::invalid_argument(std::string("Joint ") +
                                  std::to_string(joint + 1) + " must be inside (" +
                                  std::to_string(lower[joint]) + ", " +
                                  std::to_string(upper[joint]) + ") radians");
    }
    waypoint[joint] = angle;
  }
  return waypoint;
}

double segmentDuration(const Joints& start, const Joints& end) {
  double max_displacement = 0.0;
  for (std::size_t joint = 0; joint < start.size(); ++joint) {
    max_displacement = std::max(max_displacement, std::abs(end[joint] - start[joint]));
  }
  // The quintic blend has a peak normalized speed of 1.875.
  return std::max(1.0, 1.875 * max_displacement / peak_speed);
}

Joints interpolate(const Joints& start, const Joints& end, double fraction) {
  if (fraction >= 1.0) {
    return end;
  }
  // Each segment starts and ends at zero planned velocity and acceleration.
  const double blend = fraction * fraction * fraction *
                       (10.0 + fraction * (-15.0 + 6.0 * fraction));
  Joints command{};
  for (std::size_t joint = 0; joint < start.size(); ++joint) {
    command[joint] = start[joint] + blend * (end[joint] - start[joint]);
  }
  return command;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    printUsage(std::cout, argv[0]);
    return 0;
  }
  if (argc != 8) {
    printUsage(std::cerr, argv[0]);
    return 2;
  }

  Joints destination{};
  try {
    destination = parseDestination(argv + 1);
  } catch (const std::exception& error) {
    std::cerr << "Invalid destination: " << error.what() << '\n';
    return 2;
  }

  try {
    franka::Robot robot("172.16.0.2", franka::RealtimeConfig::kIgnore);
    Joints start{};
    double elapsed = 0.0;
    double duration = 0.0;

    robot.control(
        [&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
          if (duration == 0.0) {
            start = state.q_d;
            duration = segmentDuration(start, destination);
            return franka::JointPositions(start);
          }

          elapsed += period.toSec();
          const franka::JointPositions command(
              interpolate(start, destination, elapsed / duration));
          constexpr double final_hold_seconds = 0.25;
          if (elapsed >= duration + final_hold_seconds) {
            return franka::MotionFinished(command);
          }
          return command;
        },
        franka::ControllerMode::kJointImpedance, true);
    std::cout << "Joint trajectory completed.\n";
  } catch (const std::exception& error) {
    std::cerr << "Joint motion failed: " << error.what() << '\n';
    return 1;
  }
}
