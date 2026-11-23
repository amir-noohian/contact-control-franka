#pragma once

#include <franka/robot.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <stdexcept>

namespace joint_trajectory {

using Joints = std::array<double, 7>;
struct Timing {
  double peak_speed = 0.15;  // rad/s per joint
  double minimum_seconds = 1.0;
  double settle_seconds = 0.25;
};

inline double segmentDuration(const Joints& start, const Joints& end, const Timing& timing = {}) {
  double max_displacement = 0.0;
  for (std::size_t joint = 0; joint < start.size(); ++joint) {
    max_displacement = std::max(max_displacement, std::abs(end[joint] - start[joint]));
  }
  // The quintic blend has a peak normalized speed of 1.875.
  return std::max(timing.minimum_seconds, 1.875 * max_displacement / timing.peak_speed);
}

inline Joints interpolate(const Joints& start, const Joints& end, double fraction) {
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

inline void move(franka::Robot& robot, const Joints& destination,
                 const volatile std::sig_atomic_t* interrupted = nullptr,
                 const Timing& timing = {}) {
  Joints start{};
  double elapsed = 0.0;
  double duration = 0.0;

  robot.control(
      [&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
        if (interrupted && *interrupted) {
          throw std::runtime_error("Joint move interrupted");
        }
        if (duration == 0.0) {
          start = state.q_d;
          duration = segmentDuration(start, destination, timing);
          return franka::JointPositions(start);
        }

        elapsed += period.toSec();
        const franka::JointPositions command(
            interpolate(start, destination, elapsed / duration));
        if (elapsed >= duration + timing.settle_seconds) {
          return franka::MotionFinished(command);
        }
        return command;
      },
      franka::ControllerMode::kJointImpedance, true);
}

}  // namespace joint_trajectory
