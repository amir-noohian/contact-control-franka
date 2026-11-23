#include "gain_config.h"
#include "planar_dynamics.h"
#include "run_log.h"

#include <franka/model.h>
#include <franka/robot.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using Joints = std::array<double, 7>;
using namespace planar_dynamics;
constexpr Joints hold_kp{600, 0, 600, 0, 250, 150, 50};
constexpr Joints hold_kd{50, 0, 50, 0, 30, 25, 15};
constexpr Joints torque_limit{12, 12, 12, 30, 5, 5, 5};
constexpr double torque_slew_limit = 100.0;  // Nm/s
constexpr double max_offset = 0.05;           // m
constexpr double leg_seconds = 20.0;
constexpr double run_seconds = 2.0 * leg_seconds + 1.0;
constexpr double stationary_test_seconds = 6.0;
constexpr double max_tracking_error = 0.03;  // m
constexpr double x_only_z_error_limit = 0.005;  // m
constexpr double x_only_hold_seconds = 1.0;
constexpr double max_hold_error = 0.08;       // rad
constexpr double max_active_excursion = 0.262;  // rad

struct Gains {
  Vector kp = Vector::Zero();
  Vector kd = Vector::Zero();
};

Gains loadGains(const std::filesystem::path& path) {
  Gains gains;
  readGainConfig(path, {
      {"x_kp", &gains.kp[0]}, {"x_kd", &gains.kd[0]},
      {"z_kp", &gains.kp[1]}, {"z_kd", &gains.kd[1]},
  });
  return gains;
}

volatile std::sig_atomic_t interrupted = 0;

void requestStop(int) {
  interrupted = 1;
}

struct Reference {
  Vector position;
  Vector velocity;
  Vector acceleration;
};

Reference reference(double elapsed, const Vector& start, const Vector& offset) {
  const bool outbound = elapsed <= leg_seconds;
  const double s = std::clamp(
      (outbound ? elapsed : 2.0 * leg_seconds - elapsed) / leg_seconds, 0.0, 1.0);
  const double blend = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
  const double speed = (outbound ? 1.0 : -1.0) * 30.0 * s * s * (1.0 - s) * (1.0 - s) /
                       leg_seconds;
  const double acceleration = 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) /
                              (leg_seconds * leg_seconds);
  return {start + blend * offset, speed * offset, acceleration * offset};
}

bool withinJointLimits(const Joints& q) {
  // FR3 limits for joints 2 and 4 with the same 0.03 rad margin as computed_torque.
  return q[active[0]] > -1.8361 + 0.03 && q[active[0]] < 1.8361 - 0.03 &&
         q[active[1]] > -3.0770 + 0.03 && q[active[1]] < -0.1169 - 0.03;
}

struct Sample {
  double elapsed;
  Vector position;
  Vector velocity;
  Reference desired;
  Dynamics dynamics;
  Vector wrench;
  Joints q;
  Joints measured_torque;
  Joints raw_torque;
  Joints torque;
  std::array<double, 3> external_force;
  double total_load_mass;
  Joints gravity;
};

const char* runController(franka::Robot& robot, const franka::Model& model,
                          const Vector& offset, const Gains& gains, std::vector<Sample>& samples) {
  Joints start_q{};
  Vector start_position = Vector::Zero();
  PlanarJacobian previous = PlanarJacobian::Zero();
  PlanarJacobian derivative = PlanarJacobian::Zero();
  double elapsed = 0.0;
  const char* stop_reason = nullptr;
  const bool hold_only = offset.isZero(0.0);
  const bool fixed_z = offset[1] == 0.0;
  const double initial_hold = fixed_z && !hold_only ? x_only_hold_seconds : 0.0;
  const double duration = hold_only ? stationary_test_seconds : initial_hold + run_seconds;

  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
    if (!std::all_of(state.tau_J_d.begin(), state.tau_J_d.end(),
                     [](double torque) { return std::isfinite(torque); })) {
      throw std::runtime_error("Non-finite previous torque command");
    }
    const auto finish = [&](const char* reason) {
      stop_reason = reason;
      return franka::MotionFinished(franka::Torques(state.tau_J_d));
    };

    if (interrupted || samples.size() == samples.capacity()) {
      return finish(interrupted ? "Interrupted by Ctrl+C" : "Log buffer filled");
    }

    const Vector position(state.O_T_EE[12], state.O_T_EE[14]);
    const Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
    if (!position.allFinite() || !dq.allFinite() || !withinJointLimits(state.q)) {
      return finish("Invalid state or active joint reached its position margin");
    }

    const PlanarJacobian planar = planarJacobian(model.zeroJacobian(franka::Frame::kEndEffector, state));
    const Matrix jacobian = activeJacobian(planar);
    if (!planar.allFinite() || !usable(jacobian)) {
      return finish("x/z Jacobian is too close to singular");
    }

    const double dt = period.toSec();
    if (samples.empty()) {
      start_q = state.q;
      start_position = position;
    } else if (dt > 0.0) {
      elapsed += dt;
      updateJacobianDerivative(derivative, planar, previous, dt);
    }
    previous = planar;

    const auto desired = reference(elapsed - initial_hold, start_position, offset);
    if (fixed_z && std::abs(desired.position[1] - position[1]) > x_only_z_error_limit) {
      return finish("Exceeded 5 mm of z tracking error with a fixed z target");
    }
    if ((desired.position - position).norm() > max_tracking_error) {
      return finish("x/z tracking error exceeded its limit");
    }

    Joints torque{};
    for (std::size_t j = 0; j < torque.size(); ++j) {
      const bool moving = j == active[0] || j == active[1];
      const double error = start_q[j] - state.q[j];
      if (!std::isfinite(error) || std::abs(error) > (moving ? max_active_excursion : max_hold_error)) {
        return finish(moving ? "Active joint exceeded its excursion limit"
                             : "Holding joint exceeded its position error limit");
      }
      torque[j] = hold_kp[j] * error - hold_kd[j] * state.dq[j];
    }

    // Assume zero acceleration on the held joints and use all measured velocities.
    const Vector velocity = planar * dq;
    const auto dynamics = cartesianDynamics(model, state, jacobian, derivative * dq);
    const Vector acceleration = desired.acceleration +
                                gains.kp.cwiseProduct(desired.position - position) +
                                gains.kd.cwiseProduct(desired.velocity - velocity);
    const Vector wrench = dynamics.inertia * acceleration + dynamics.bias;
    const Vector active_torque = jacobian.transpose() * wrench;
    // franka::Torques accepts commands without gravity. The robot adds it.
    torque[active[0]] = active_torque[0];
    torque[active[1]] = active_torque[1];
    const Joints raw_torque = torque;

    for (std::size_t j = 0; j < torque.size(); ++j) {
      const double previous_torque = state.tau_J_d[j];
      if (!std::isfinite(torque[j])) {
        return finish("Non-finite torque");
      }
      if (std::abs(previous_torque) > torque_limit[j]) {
        return finish("Previous torque command exceeded the configured limit");
      }
      const double step = torque_slew_limit * std::min(0.001, dt);
      const double bounded = std::clamp(torque[j], -torque_limit[j], torque_limit[j]);
      torque[j] = std::clamp(bounded, previous_torque - step, previous_torque + step);
    }
    // Reverse the robot's reaction convention to report force on the tool.
    const auto& external = state.O_F_ext_hat_K;
    // Record modeled gravity for the CSV. The FCI still receives only torque.
    samples.push_back({elapsed, position, velocity, desired, dynamics, wrench, state.q,
                       state.tau_J, raw_torque, torque,
                       {-external[0], -external[1], -external[2]}, state.m_total,
                       model.gravity(state)});

    const franka::Torques command(torque);
    return elapsed >= duration ? franka::MotionFinished(command) : command;
  }, true);
  return stop_reason;
}

void writeLog(std::ostream& out, const std::vector<Sample>& samples) {
  out << std::setprecision(17)
      << "elapsed_s,x_m,z_m,dx_m_s,dz_m_s,x_ref_m,z_ref_m,dx_ref_m_s,dz_ref_m_s,"
         "ddx_ref_m_s2,ddz_ref_m_s2,inertia_xx_kg,inertia_xz_kg,inertia_zx_kg,inertia_zz_kg,"
         "bias_x_N,bias_z_N,wrench_x_N,wrench_z_N";
  for (const char* label : {"q_rad", "tau_command_Nm", "tau_measured_Nm", "tau_raw_Nm"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      out << ',' << label << '_' << joint;
    }
  }
  out << ",external_fx_N,external_fy_N,external_fz_N,configured_total_load_kg";
  for (const char* label : {"tau_gravity_Nm", "tau_command_plus_gravity_Nm"}) {
    for (int joint = 1; joint <= 7; ++joint) {
      out << ',' << label << '_' << joint;
    }
  }
  out << '\n';
  for (const auto& s : samples) {
    out << s.elapsed;
    for (const auto* value : {&s.position, &s.velocity, &s.desired.position, &s.desired.velocity,
                              &s.desired.acceleration}) {
      out << ',' << (*value)[0] << ',' << (*value)[1];
    }
    const auto& inertia = s.dynamics.inertia;
    out << ',' << inertia(0, 0) << ',' << inertia(0, 1) << ',' << inertia(1, 0)
        << ',' << inertia(1, 1) << ',' << s.dynamics.bias[0] << ',' << s.dynamics.bias[1]
        << ',' << s.wrench[0] << ',' << s.wrench[1];
    for (const auto* values : {&s.q, &s.torque, &s.measured_torque, &s.raw_torque}) {
      for (double value : *values) {
        out << ',' << value;
      }
    }
    for (double force : s.external_force) {
      out << ',' << force;
    }
    out << ',' << s.total_load_mass;
    for (double value : s.gravity) {
      out << ',' << value;
    }
    for (std::size_t joint = 0; joint < s.torque.size(); ++joint) {
      out << ',' << s.torque[joint] + s.gravity[joint];
    }
    out << '\n';
  }
}

void printUsage(std::ostream& out, const char* program) {
  out << "Usage: " << program << " DX DZ\n"
      << "Track a base-frame x/z offset in meters and return to the starting pose.\n"
      << "The move takes " << leg_seconds << " seconds each way.\n"
      << "Use 0 0 to hold the starting x/z target for six seconds.\n"
      << "Offset magnitude must be at most 0.05 m.\n"
      << "Connects to 172.16.0.2 and saves a CSV.\n"
      << "Reads tracking gains from config/cartesian_computed_torque.conf.\n";
}

double parseOffset(const char* arg) {
  std::size_t length = 0;
  const std::string text(arg);
  const double value = std::stod(text, &length);
  if (length != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("Offsets must be finite numbers");
  }
  return value;
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc == 2 && std::string_view(argv[1]) == "--help") {
    printUsage(std::cout, argv[0]);
    return 0;
  }
  if (argc != 3) {
    printUsage(std::cerr, argv[0]);
    return 2;
  }

  Vector offset;
  try {
    offset = Vector(parseOffset(argv[1]), parseOffset(argv[2]));
    if (offset.stableNorm() > max_offset) {
      throw std::invalid_argument("Offset magnitude must be at most 0.05 m");
    }
  } catch (const std::exception& error) {
    std::cerr << "Invalid offset: " << error.what() << '\n';
    return 2;
  }

  try {
    const auto config_path = std::filesystem::path(FRANKA_PROJECT_DIR) /
                             "config" / "cartesian_computed_torque.conf";
    const auto gains = loadGains(config_path);
    std::cout << "x gains: Kp=" << gains.kp[0] << " Kd=" << gains.kd[0] << '\n'
              << "z gains: Kp=" << gains.kp[1] << " Kd=" << gains.kd[1] << std::endl;
    if (std::signal(SIGINT, requestStop) == SIG_ERR) {
      throw std::runtime_error("Could not register Ctrl+C handler");
    }
    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>((run_seconds + x_only_hold_seconds) * 1000) + 100);

    franka::Robot robot("172.16.0.2", franka::RealtimeConfig::kIgnore);
    const auto model = robot.loadModel();
    const auto initial = robot.readOnce();
    if (!withinJointLimits(initial.q) || !usable(activeJacobian(planarJacobian(model.zeroJacobian(franka::Frame::kEndEffector, initial))))) {
      throw std::runtime_error("Starting pose is too close to an active joint limit or x/z singularity");
    }

    const auto path = reserveLogPath(std::filesystem::path(FRANKA_PROJECT_DIR) /
                                     "data" / "cartesian_computed_torque");
    std::ofstream log(path);
    if (!log) {
      throw std::runtime_error("Could not open " + path.string());
    }

    int result = 0;
    try {
      if (const char* reason = runController(robot, model, offset, gains, samples)) {
        std::cerr << "Controller stopped: " << reason << '\n';
        result = 1;
      }
    } catch (const std::exception& error) {
      std::cerr << "Control failed: " << error.what() << '\n';
      result = 1;
    }

    writeLog(log, samples);
    log.close();
    if (!log) {
      throw std::runtime_error("Could not save complete CSV to " + path.string());
    }
    std::cout << "Saved " << samples.size() << " samples to " << path << '\n';
    return result;
  } catch (const std::exception& error) {
    std::cerr << "Cartesian computed torque failed: " << error.what() << '\n';
    return 1;
  }
}
