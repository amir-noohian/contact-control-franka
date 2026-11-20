#include <franka/robot.h>
#include <franka/exception.h>

#include <iostream>

int main() {
  try {
    // Connect to Franka through FCI
    franka::Robot robot(
        "172.16.0.2",
        franka::RealtimeConfig::kIgnore
    );

    std::cout << "Connected to Franka!" << std::endl;

    // Read robot state once (no motion)
    franka::RobotState state = robot.readOnce();

    std::cout << "\nJoint positions (rad):\n";

    for (size_t i = 0; i < state.q.size(); ++i) {
      std::cout << "Joint " << i + 1
                << ": " << state.q[i] << std::endl;
    }

  } catch (const franka::Exception& e) {
    std::cerr << "Franka error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
