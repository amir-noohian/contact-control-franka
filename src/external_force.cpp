#include <franka/robot.h>

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int) {
  stop_requested = 1;
}

void printUsage(std::ostream& out, const char* program) {
  out << "Usage: " << program << " [robot-host [interval-ms]]\n"
      << "Continuously display estimated force and torque in base and end-effector axes.\n"
      << "Also displays estimated external joint torques for J1 through J7.\n"
      << "Defaults: robot-host=172.16.0.2, interval-ms=100. Ctrl+C stops the stream.\n";
}

using Vector3 = std::array<double, 3>;

Vector3 rotateToEndEffector(const std::array<double, 16>& base_T_ee, const Vector3& base) {
  Vector3 ee{};
  for (std::size_t axis = 0; axis < ee.size(); ++axis) {
    // The columns of the pose are the end-effector axes expressed in base axes.
    const auto offset = 4 * axis;
    ee[axis] = base_T_ee[offset] * base[0] + base_T_ee[offset + 1] * base[1] +
               base_T_ee[offset + 2] * base[2];
  }
  return ee;
}

Vector3 torqueAboutEndEffector(const std::array<double, 16>& ee_T_k, const Vector3& force,
                               const Vector3& torque_about_k) {
  // Shift the moment from K to EE using the vector from the EE origin to K.
  const double x = ee_T_k[12];
  const double y = ee_T_k[13];
  const double z = ee_T_k[14];
  return {torque_about_k[0] + y * force[2] - z * force[1],
          torque_about_k[1] + z * force[0] - x * force[2],
          torque_about_k[2] + x * force[1] - y * force[0]};
}

void printWrench(std::ostream& out, const char* frame, const Vector3& force,
                 const Vector3& torque) {
  out << std::left << std::setw(22) << frame << std::right;
  for (double component : force) {
    out << std::setw(12) << component;
  }
  for (double component : torque) {
    out << std::setw(12) << component;
  }
  out << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
  const std::string_view host = argc > 1 ? argv[1] : "172.16.0.2";
  if (argc == 2 && host == "--help") {
    printUsage(std::cout, argv[0]);
    return 0;
  }
  if (argc > 3 || host.empty() || host.front() == '-') {
    printUsage(std::cerr, argv[0]);
    return 2;
  }

  int interval_ms = 100;
  if (argc == 3) {
    const std::string_view value(argv[2]);
    const auto* end = value.data() + value.size();
    const auto parsed = std::from_chars(value.data(), end, interval_ms);
    if (parsed.ec != std::errc{} || parsed.ptr != end || interval_ms <= 0) {
      std::cerr << "interval-ms must be a positive integer.\n";
      return 2;
    }
  }

  if (std::signal(SIGINT, requestStop) == SIG_ERR) {
    std::cerr << "Could not register the Ctrl+C handler.\n";
    return 1;
  }

  try {
    franka::Robot robot(host.data(), franka::RealtimeConfig::kIgnore);
    using Clock = std::chrono::steady_clock;
    const auto interval = std::chrono::milliseconds(interval_ms);
    auto next_print = Clock::time_point::min();

    std::cout << "Estimated wrench acting on the tool\n"
              << std::left << std::setw(22) << "Axes / moment origin" << std::right
              << std::setw(12) << "Fx (N)" << std::setw(12) << "Fy (N)"
              << std::setw(12) << "Fz (N)" << std::setw(12) << "Tx (N m)"
              << std::setw(12) << "Ty (N m)" << std::setw(12) << "Tz (N m)" << '\n'
              << std::fixed << std::setprecision(3);
    robot.read([&](const franka::RobotState& state) {
      if (stop_requested || !std::cout) {
        return false;
      }
      const auto now = Clock::now();
      if (now >= next_print) {
        next_print = now + interval;
        const auto& wrench = state.O_F_ext_hat_K;
        // Reverse Franka's reaction convention to report the wrench on the tool.
        const Vector3 force_base{-wrench[0], -wrench[1], -wrench[2]};
        const Vector3 torque_base{-wrench[3], -wrench[4], -wrench[5]};
        const auto force_ee = rotateToEndEffector(state.O_T_EE, force_base);
        const auto torque_ee_about_k = rotateToEndEffector(state.O_T_EE, torque_base);
        const auto torque_ee = torqueAboutEndEffector(state.EE_T_K, force_ee, torque_ee_about_k);

        std::cout << "\nRobot time " << state.time.toMSec() << " ms | Force magnitude "
                  << std::hypot(force_base[0], force_base[1], force_base[2]) << " N\n";
        printWrench(std::cout, "Base / K", force_base, torque_base);
        printWrench(std::cout, "End effector / EE", force_ee, torque_ee);
        std::cout << "Estimated external joint torque (N m)";
        for (std::size_t joint = 0; joint < state.tau_ext_hat_filtered.size(); ++joint) {
          std::cout << "  J" << joint + 1 << '=' << state.tau_ext_hat_filtered[joint];
        }
        std::cout << '\n' << std::flush;
      }
      return !stop_requested && std::cout.good();
    });
    if (!std::cout) {
      std::cerr << "Could not write force readings.\n";
      return 1;
    }
  } catch (const std::exception& error) {
    std::cerr << "External force stream failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
