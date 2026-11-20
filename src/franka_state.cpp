#include <franka/model.h>
#include <franka/robot.h>

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

namespace {

using franka::Frame;

struct NamedFrame {
  const char* name;
  Frame frame;
};

constexpr NamedFrame frames[] = {
    {"joint1", Frame::kJoint1}, {"joint2", Frame::kJoint2},
    {"joint3", Frame::kJoint3}, {"joint4", Frame::kJoint4},
    {"joint5", Frame::kJoint5}, {"joint6", Frame::kJoint6},
    {"joint7", Frame::kJoint7}, {"flange", Frame::kFlange},
    {"end_effector", Frame::kEndEffector}, {"stiffness", Frame::kStiffness},
};

void printUsage(std::ostream& out, const char* program) {
  out << "Usage: " << program << " [--json] [robot-host [samples [interval-ms]]]\n"
      << "       " << program << " --urdf robot-host\n"
      << "       " << program << " --help\n"
      << "Defaults: robot-host=172.16.0.2, samples=1, interval-ms=100\n";
}

int positiveInteger(std::string_view value, const char* name) {
  int result = 0;
  const auto* end = value.data() + value.size();
  const auto parsed = std::from_chars(value.data(), end, result);
  if (parsed.ec != std::errc{} || parsed.ptr != end || result <= 0) {
    throw std::invalid_argument(std::string(name) + " must be a positive integer");
  }
  return result;
}

template <std::size_t N>
void writeArray(std::ostream& out, const std::array<double, N>& values) {
  out << '[';
  for (std::size_t i = 0; i < N; ++i) {
    if (i != 0) {
      out << ',';
    }
    if (std::isfinite(values[i])) {
      out << values[i];
    } else {
      out << "null";
    }
  }
  out << ']';
}

template <std::size_t Rows, std::size_t N>
void printMatrix(std::ostream& out, const char* label, const std::array<double, N>& values) {
  static_assert(N % Rows == 0);
  out << "  " << label << '\n';
  for (std::size_t row = 0; row < Rows; ++row) {
    out << "   ";
    for (std::size_t column = 0; column < N / Rows; ++column) {
      out << ' ' << std::setw(11) << values[column * Rows + row];
    }
    out << '\n';
  }
}

void printSample(std::ostream& out, int sample, const franka::RobotState& state,
                 const franka::Model& model) {
  out << std::fixed << std::setprecision(5)
      << "\nSample " << sample << " | Robot time " << state.time.toMSec()
      << " ms | Mode " << state.robot_mode << "\n";
  out << "\nJoint state\n";
  printMatrix<1>(out, "Position (rad)", state.q);
  printMatrix<1>(out, "Velocity (rad/s)", state.dq);
  printMatrix<1>(out, "Measured torque (Nm)", state.tau_J);
  printMatrix<1>(out, "Estimated external torque (Nm)", state.tau_ext_hat_filtered);
  printMatrix<1>(out, "Estimated external wrench in base axes (N, Nm)", state.O_F_ext_hat_K);
  printMatrix<1>(out, "Joint contact flags", state.joint_contact);
  printMatrix<1>(out, "Joint collision flags", state.joint_collision);
  out << "  Current errors " << state.current_errors << '\n';

  out << "\nDynamics | Configured tool and payload mass " << state.m_total << " kg\n";
  printMatrix<7>(out, "Joint-space mass matrix", model.mass(state));
  printMatrix<1>(out, "Coriolis and centrifugal torque (Nm)", model.coriolis(state));
  printMatrix<1>(out, "Gravity torque (Nm)", model.gravity(state));

  for (const auto& [name, frame] : frames) {
    out << "\nFrame " << name << '\n';
    printMatrix<4>(out, "Pose relative to base", model.pose(frame, state));
    printMatrix<6>(out, "Base-frame Jacobian", model.zeroJacobian(frame, state));
    printMatrix<6>(out, "Body-frame Jacobian", model.bodyJacobian(frame, state));
  }
  out << std::flush;
  if (!out) {
    throw std::runtime_error("Could not write robot state output");
  }
}

void writeSample(std::ostream& out, const franka::RobotState& state,
                 const franka::Model& model, franka::Robot::ServerVersion version) {
  // Buffer each complete record before writing it to the output stream.
  std::ostringstream record;
  record.imbue(std::locale::classic());
  record << std::setprecision(std::numeric_limits<double>::max_digits10);
  record << "{\"server_version\":" << version << ",\"state\":" << state
         << ",\"dynamics\":{\"mass\":";
  writeArray(record, model.mass(state));
  record << ",\"coriolis\":";
  writeArray(record, model.coriolis(state));
  record << ",\"gravity\":";
  writeArray(record, model.gravity(state));
  record << "},\"frames\":{";

  const char* separator = "";
  for (const auto& [name, frame] : frames) {
    record << separator << '"' << name << "\":{\"pose\":";
    writeArray(record, model.pose(frame, state));
    record << ",\"zero_jacobian\":";
    writeArray(record, model.zeroJacobian(frame, state));
    record << ",\"body_jacobian\":";
    writeArray(record, model.bodyJacobian(frame, state));
    record << '}';
    separator = ",";
  }
  record << "}}\n";
  out << record.str() << std::flush;
  if (!out) {
    throw std::runtime_error("Could not write robot state output");
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  const bool json = argc > 1 && std::string_view(argv[1]) == "--json";
  const int host_index = json ? 2 : 1;
  const std::string_view first = argc > host_index ? argv[host_index] : "172.16.0.2";
  if (argc == 2 && first == "--help") {
    printUsage(std::cout, argv[0]);
    return 0;
  }

  const bool urdf = !json && first == "--urdf";
  if ((urdf && argc != 3) ||
      (!urdf && (argc > host_index + 3 || first.empty() || first.front() == '-'))) {
    printUsage(std::cerr, argv[0]);
    return 2;
  }

  try {
    if (urdf) {
      franka::Robot robot(argv[2], franka::RealtimeConfig::kIgnore);
      std::cout << robot.getRobotModel() << std::endl;
      return std::cout ? 0 : 1;
    }

    const int samples = argc > host_index + 1 ? positiveInteger(argv[host_index + 1], "samples") : 1;
    const auto interval =
        std::chrono::milliseconds(argc > host_index + 2 ? positiveInteger(argv[host_index + 2], "interval-ms")
                                             : 100);
    franka::Robot robot(std::string(first), franka::RealtimeConfig::kIgnore);
    const auto model = robot.loadModel();
    const auto version = robot.serverVersion();

    for (int i = 0; i < samples; ++i) {
      const auto next_sample = std::chrono::steady_clock::now() + interval;
      const auto state = robot.readOnce();
      if (json) {
        writeSample(std::cout, state, model, version);
      } else {
        printSample(std::cout, i + 1, state, model);
      }
      if (i + 1 < samples) {
        std::this_thread::sleep_until(next_sample);
      }
    }
  } catch (const std::invalid_argument& error) {
    std::cerr << error.what() << '\n';
    printUsage(std::cerr, argv[0]);
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "Franka state read failed: " << error.what() << '\n';
    return 1;
  }
}
