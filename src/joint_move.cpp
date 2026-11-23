#include "joint_trajectory.h"

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
    joint_trajectory::move(robot, destination);
    std::cout << "Joint trajectory completed.\n";
  } catch (const std::exception& error) {
    std::cerr << "Joint motion failed: " << error.what() << '\n';
    return 1;
  }
}
