#include "cartesian_force_program.h"
#include "gain_config.h"
#include "iac1_control.h"
#include "joint_trajectory.h"
#include "planar_dynamics.h"
#include "run_log.h"

#include <franka/model.h>
#include <franka/robot.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <poll.h>
#include <unistd.h>

namespace {

using Joints = std::array<double, 7>;
constexpr int joint2 = 1;
constexpr int joint4 = 3;

struct ContactSettings {
  std::string robot_host;
  double ignore_realtime{};
  Joints contact_pose{}, hold_kp{}, hold_kd{}, torque_limit{};
  joint_trajectory::Timing joint_move;
  double pose_tolerance{}, joint_2_min{}, joint_2_max{}, joint_4_min{}, joint_4_max{};
  double active_excursion{}, held_excursion{}, torque_rate{};
  double min_target{}, max_target{}, max_force_command{}, max_measured{}, min_contact{};
  double ramp_seconds{}, contact_timeout{}, no_contact_travel{}, contact_confirmation{};
  double integral_force_limit{}, max_tool_travel{}, max_z_speed{}, min_z_jacobian_norm{};
  double duration{}, contact_loss{};
};

void loadContactSettings(const std::filesystem::path& path, ContactSettings& settings,
                         std::map<std::string, double*> keys = {},
                         std::set<std::string> signed_keys = {}, bool force_integral = true) {
  keys.insert({
      {"ignore_realtime", &settings.ignore_realtime},
      {"pose_tolerance_rad", &settings.pose_tolerance},
      {"joint_move_peak_rad_s", &settings.joint_move.peak_speed},
      {"joint_move_min_s", &settings.joint_move.minimum_seconds},
      {"joint_move_settle_s", &settings.joint_move.settle_seconds},
      {"joint_2_min_rad", &settings.joint_2_min}, {"joint_2_max_rad", &settings.joint_2_max},
      {"joint_4_min_rad", &settings.joint_4_min}, {"joint_4_max_rad", &settings.joint_4_max},
      {"active_excursion_rad", &settings.active_excursion}, {"held_excursion_rad", &settings.held_excursion},
      {"torque_rate_Nm_s", &settings.torque_rate},
      {"min_target_N", &settings.min_target}, {"max_target_N", &settings.max_target},
      {"max_force_command_N", &settings.max_force_command},
      {"max_measured_force_N", &settings.max_measured},
      {"contact_threshold_N", &settings.min_contact},
      {"contact_confirmation_s", &settings.contact_confirmation},
      {"contact_timeout_s", &settings.contact_timeout},
      {"no_contact_travel_m", &settings.no_contact_travel},
      {"force_ramp_s", &settings.ramp_seconds},
      {"max_tool_travel_m", &settings.max_tool_travel},
      {"max_z_speed_m_s", &settings.max_z_speed},
      {"min_z_jacobian_norm_m_rad", &settings.min_z_jacobian_norm},
  });
  if (force_integral) {
    keys.emplace("force_integral_limit_N", &settings.integral_force_limit);
  }
  signed_keys.insert({
      "ignore_realtime", "joint_2_min_rad", "joint_2_max_rad", "joint_4_min_rad", "joint_4_max_rad",
  });
  for (int joint = 0; joint < 7; ++joint) {
    const auto id = std::to_string(joint + 1);
    const auto pose_key = "contact_q_" + id;
    keys.emplace(pose_key, &settings.contact_pose[joint]);
    signed_keys.insert(pose_key);
    keys.emplace("torque_limit_" + id + "_Nm", &settings.torque_limit[joint]);
    if (joint != joint2 && joint != joint4) {
      keys.emplace("hold_kp_" + id, &settings.hold_kp[joint]);
      keys.emplace("hold_kd_" + id, &settings.hold_kd[joint]);
    }
  }
  readGainConfig(path, keys, signed_keys, {{"robot_host", &settings.robot_host}});
}

bool validJointSettings(const ContactSettings& settings) {
  // Hardware position bounds stay fixed. Configured operating bounds must fit inside them.
  constexpr Joints lower{-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508};
  constexpr Joints upper{2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
  constexpr Joints hardware_torque{87, 87, 87, 87, 12, 12, 12};
  for (std::size_t j = 0; j < lower.size(); ++j) {
    if (settings.contact_pose[j] <= lower[j] || settings.contact_pose[j] >= upper[j] ||
        settings.torque_limit[j] > hardware_torque[j]) {
      return false;
    }
  }
  return lower[joint2] <= settings.joint_2_min &&
         settings.joint_2_min < settings.contact_pose[joint2] &&
         settings.contact_pose[joint2] < settings.joint_2_max &&
         settings.joint_2_max <= upper[joint2] &&
         lower[joint4] <= settings.joint_4_min &&
         settings.joint_4_min < settings.contact_pose[joint4] &&
         settings.contact_pose[joint4] < settings.joint_4_max &&
         settings.joint_4_max <= upper[joint4];
}

volatile std::sig_atomic_t interrupted = 0;

void requestStop(int) {
  interrupted = 1;
}

bool atContactPose(const Joints& position, const ContactSettings& settings) {
  for (std::size_t joint = 0; joint < position.size(); ++joint) {
    if (!std::isfinite(position[joint]) ||
        std::abs(position[joint] - settings.contact_pose[joint]) > settings.pose_tolerance) {
      return false;
    }
  }
  return true;
}

char readCommand() {
  std::string line;
  pollfd input{STDIN_FILENO, POLLIN, 0};
  while (!interrupted) {
    const int ready = poll(&input, 1, 100);
    if (ready < 0 && errno != EINTR) {
      throw std::runtime_error("Could not poll command input");
    }
    if (ready > 0 && !interrupted) {
      char key = '\0';
      const auto count = read(STDIN_FILENO, &key, 1);
      if (count == 0) {
        return 'q';
      }
      if (count < 0 && errno != EINTR) {
        throw std::runtime_error("Could not read command input");
      }
      if (count == 1 && key == '\n') {
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || first != line.find_last_not_of(" \t\r")) {
          return '\0';
        }
        return line[first];
      }
      if (count == 1) {
        line += key;
      }
    }
  }
  return 'q';
}

struct Gains {
  double kp = 0.0;
  double ki = 0.0;
};

struct HybridMotion {
  double offset = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  double ki = 0.0;
  double max_acceleration = 0.0;
  double joint4_excursion = 0.0;
  double max_offset = 0.0;
  double motion_seconds = 0.0;
  double hold_seconds = 0.0;
  double max_error = 0.0;
  double max_speed = 0.0;
  double max_integral = 0.0;
  double min_singular_value = 0.0;
  double jacobian_filter_time = 0.0;
};

struct Iac1Settings {
  planar_dynamics::Vector target_offset = planar_dynamics::Vector::Zero();
  double nominal_angle{}, initial_angle{}, max_angle_offset{}, gamma{};
  double kp{}, kv{}, max_acceleration{}, difference_step{}, min_singular_value{};
  double max_x_speed{}, max_target_offset{}, max_task_wrench{};
};

struct Iac1Sample {
  planar_dynamics::Vector target = planar_dynamics::Vector::Zero();
  double normal_force{}, estimate{}, effective_angle{}, estimator_rate{};
};

struct MotionReference {
  double x;
  double dx;
  double ddx;
};

MotionReference motionReference(double start_x, double offset, double time, double motion_seconds) {
  const double s = std::clamp(time / motion_seconds, 0.0, 1.0);
  return {
      start_x + offset * s * s * s * (10.0 + s * (-15.0 + 6.0 * s)),
      offset * 30.0 * s * s * (1.0 - s) * (1.0 - s) / motion_seconds,
      offset * 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) /
          (motion_seconds * motion_seconds),
  };
}

struct ExternalEstimate {
  std::array<double, 6> wrench;
  Joints joint_torque;

  bool isFinite() const {
    const auto finite = [](double value) { return std::isfinite(value); };
    return std::all_of(wrench.begin(), wrench.end(), finite) &&
           std::all_of(joint_torque.begin(), joint_torque.end(), finite);
  }
};

ExternalEstimate externalEstimate(const franka::RobotState& state) {
  ExternalEstimate estimate{state.O_F_ext_hat_K, state.tau_ext_hat_filtered};
  // Report the base-frame wrench exerted by the environment on the robot.
  for (double& value : estimate.wrench) {
    value = -value;
  }
  return estimate;
}

ExternalEstimate zeroedEstimate(ExternalEstimate estimate, const ExternalEstimate& baseline) {
  for (std::size_t i = 0; i < estimate.wrench.size(); ++i) {
    estimate.wrench[i] -= baseline.wrench[i];
  }
  for (std::size_t j = 0; j < estimate.joint_torque.size(); ++j) {
    estimate.joint_torque[j] -= baseline.joint_torque[j];
  }
  return estimate;
}

struct Sample {
  double time;
  double x;
  double z;
  double reference;
  ExternalEstimate external;
  double integral_force;
  double command_z;
  Joints command;
  bool in_contact;
  MotionReference motion_reference;
  double dx;
  double x_acceleration;
  double command_x;
  double total_command_z;
  Joints q;
  Iac1Sample adaptive;
};

struct JointExcursion {
  std::size_t joint = 0;
  double displacement = 0.0;
  double limit = 0.0;
};

bool withinJointLimits(const Joints& q, const ContactSettings& settings) {
  return q[joint2] > settings.joint_2_min && q[joint2] < settings.joint_2_max &&
         q[joint4] > settings.joint_4_min && q[joint4] < settings.joint_4_max;
}

double rampForce(double elapsed, double initial, double target, double ramp_seconds) {
  const double s = std::clamp(elapsed / ramp_seconds, 0.0, 1.0);
  const double blend = s * s * (3.0 - 2.0 * s);
  return initial + blend * (target - initial);
}

const char* run(franka::Robot& robot, const franka::Model& model, const Gains& gains,
                const ContactSettings& settings,
                double target, const ExternalEstimate& baseline, std::vector<Sample>& samples,
                const HybridMotion* hybrid, const Iac1Settings* adaptive, JointExcursion& excursion) {
  using namespace planar_dynamics;
  Joints start{};
  double start_x = 0.0;
  double start_z = 0.0;
  double preload = 0.0;
  double elapsed = 0.0;
  double contact_time = 0.0;
  bool in_contact = false;
  double integral_force = 0.0;
  double x_integral = 0.0;
  PlanarJacobian previous = PlanarJacobian::Zero();
  PlanarJacobian derivative = PlanarJacobian::Zero();
  const Vector nominal_tangent = adaptive ? iac1::tangent(adaptive->nominal_angle) : Vector::Zero();
  const Vector nominal_normal = adaptive ? iac1::normal(adaptive->nominal_angle) : Vector::Zero();
  double iac_start = -1.0;
  Iac1Sample iac;
  const char* reason = nullptr;

  robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
    if (!std::all_of(state.tau_J_d.begin(), state.tau_J_d.end(),
                     [](double torque) { return std::isfinite(torque); })) {
      throw std::runtime_error("Non-finite previous torque");
    }
    const auto finish = [&](const char* message) {
      reason = message;
      return franka::MotionFinished(franka::Torques(state.tau_J_d));
    };

    if (interrupted || samples.size() == samples.capacity()) {
      return finish(interrupted ? "Interrupted by Ctrl+C" : "Log buffer filled");
    }

    const auto external = zeroedEstimate(externalEstimate(state), baseline);
    const Vector measured_planar(external.wrench[0], external.wrench[2]);
    const double measured = adaptive ? measured_planar.norm() : external.wrench[2];
    const double contact_signal = adaptive ? nominal_normal.dot(measured_planar) : measured;
    if (!external.isFinite() || !std::isfinite(measured) || std::abs(measured) > settings.max_measured) {
      return finish("Estimated contact force is outside the operating range");
    }
    const double x = state.O_T_EE[12];
    const double z = state.O_T_EE[14];
    const double dt = period.toSec();
    if (samples.empty()) {
      if (!atContactPose(state.q, settings)) {
        return finish("Arm is outside the starting pose tolerance");
      }
      start = state.q;
      start_x = x;
      start_z = z;
      preload = std::clamp(contact_signal, 0.0, settings.max_force_command);
    } else {
      elapsed += dt;
    }

    const bool outside_workspace = hybrid
        ? std::abs(z - start_z) > settings.max_tool_travel ||
              x < start_x + std::min(0.0, hybrid->offset) - hybrid->max_error ||
              x > start_x + std::max(0.0, hybrid->offset) + hybrid->max_error
        : std::hypot(x - start_x, z - start_z) > settings.max_tool_travel;
    if (!std::isfinite(x) || !std::isfinite(z) || outside_workspace ||
        !withinJointLimits(state.q, settings)) {
      return finish("End effector or active joints are outside their operating bounds");
    }

    const auto jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
    const double j2z = jacobian[6 * joint2 + 2];
    const double j4z = jacobian[6 * joint4 + 2];
    if (!std::isfinite(j2z) || !std::isfinite(j4z) || std::hypot(j2z, j4z) < settings.min_z_jacobian_norm) {
      return finish("Active joints have insufficient z force authority");
    }

    if (!in_contact) {
      contact_time = contact_signal >= settings.min_contact ? contact_time + dt : 0.0;
      if (contact_time >= settings.contact_confirmation) {
        in_contact = true;
      } else if (elapsed >= settings.contact_timeout ||
                 (adaptive ? std::hypot(x - start_x, z - start_z) : std::abs(z - start_z))
                     >= settings.no_contact_travel) {
        return finish("Force control reached its travel or time limit without contact");
      }
    }

    const double unload_time = hybrid
        ? std::max(0.0, elapsed - hybrid->motion_seconds - hybrid->hold_seconds)
        : 0.0;
    if (in_contact && measured < settings.contact_loss) {
      return finish(hybrid && unload_time > 0.0 ? nullptr : "Contact lost during force control");
    }
    const double reference = adaptive && in_contact
        ? target
        : hybrid && unload_time > 0.0
        ? rampForce(unload_time, target, preload, settings.ramp_seconds)
        : rampForce(hybrid || adaptive ? elapsed : std::min(elapsed, settings.duration - elapsed), preload,
                    target, settings.ramp_seconds);
    const double error = reference - measured;
    const double raw_force = reference + gains.kp * error + integral_force;
    if (!std::isfinite(raw_force)) {
      return finish("Non-finite force command");
    }
    double velocity_z = 0.0;
    for (std::size_t j = 0; j < state.dq.size(); ++j) {
      velocity_z += jacobian[6 * j + 2] * state.dq[j];
    }
    if (!std::isfinite(velocity_z) || std::abs(velocity_z) > settings.max_z_speed) {
      return finish("Exceeded the vertical speed limit");
    }

    Matrix task_jacobian = Matrix::Zero();
    task_jacobian.row(1) << j2z, j4z;
    Vector wrench = Vector::Zero();
    MotionReference desired_x{start_x, 0.0, 0.0};
    double dx = 0.0;
    double x_acceleration = 0.0;
    double next_x_integral = x_integral;
    if (hybrid) {
      const PlanarJacobian planar = planarJacobian(jacobian);
      task_jacobian = activeJacobian(planar);
      if (!planar.allFinite() || !usable(task_jacobian, hybrid->min_singular_value)) {
        return finish("x/z Jacobian is too close to singular");
      }
      if (!samples.empty() && dt > 0.0) {
        updateJacobianDerivative(derivative, planar, previous, dt, hybrid->jacobian_filter_time);
      }
      previous = planar;
      const Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
      dx = (planar * dq).x();
      desired_x = motionReference(start_x, hybrid->offset, elapsed, hybrid->motion_seconds);
      const double x_error = desired_x.x - x;
      if (!std::isfinite(dx)) {
        return finish("Invalid measured x velocity");
      }
      if (std::abs(x_error) > hybrid->max_error) {
        return finish("Exceeded the x tracking error limit");
      }
      if (std::abs(dx) > hybrid->max_speed) {
        return finish("Exceeded the x speed limit");
      }
      const double pd = desired_x.ddx + hybrid->kp * x_error + hybrid->kd * (desired_x.dx - dx);
      if (elapsed >= hybrid->motion_seconds) {
        const double trial = std::clamp(x_integral + hybrid->ki * dt * x_error,
                                        -hybrid->max_integral, hybrid->max_integral);
        if (!((pd + trial > hybrid->max_acceleration && trial > x_integral) ||
              (pd + trial < -hybrid->max_acceleration && trial < x_integral))) {
          next_x_integral = trial;
        }
      }
      x_acceleration = std::clamp(pd + next_x_integral, -hybrid->max_acceleration,
                                 hybrid->max_acceleration);
      const auto dynamics = cartesianDynamics(model, state, task_jacobian, derivative * dq);
      wrench.x() = dynamics.inertia(0, 0) * x_acceleration + dynamics.bias.x();
      if (!std::isfinite(pd) || !std::isfinite(wrench.x())) {
        return finish("Invalid hybrid inverse dynamics");
      }
    }

    double command_z = -std::clamp(raw_force, 0.0, settings.max_force_command);
    if (adaptive) {
      const PlanarJacobian planar = planarJacobian(jacobian);
      task_jacobian = activeJacobian(planar);
      if (!planar.allFinite() || !usable(task_jacobian, adaptive->min_singular_value)) {
        return finish("IAC1 x/z Jacobian is too close to singular");
      }
      const Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(state.dq.data());
      const double measured_dx = (planar * dq).x();
      if (!std::isfinite(measured_dx) || std::abs(measured_dx) > adaptive->max_x_speed) {
        return finish("IAC1 x speed limit exceeded");
      }
      iac.normal_force = measured;
      const Vector position(x, z);
      Vector used_normal = nominal_normal;
      if (in_contact) {
        const Vector active_velocity(state.dq[joint2], state.dq[joint4]);
        const Vector velocity = task_jacobian * active_velocity;
        iac1::ModelRates rates;
        if (!iac1::modelRates(model, state, active_velocity, adaptive->difference_step,
                             adaptive->min_singular_value, rates)) {
          return finish("IAC1 model derivatives are invalid");
        }
        auto reduced_state = state;
        reduced_state.dq.fill(0.0);
        reduced_state.dq[joint2] = active_velocity.x();
        reduced_state.dq[joint4] = active_velocity.y();
        const auto dynamics = cartesianDynamics(model, reduced_state, task_jacobian,
                                                 rates.jacobian_bias);
        const double correction = iac1::momentumCorrection(
            adaptive->gamma, nominal_tangent, dynamics.inertia, velocity);
        if (iac_start < 0.0) {
          iac_start = elapsed;
          iac.target = position + adaptive->target_offset;
          iac.estimate = adaptive->initial_angle + correction;
        }
        iac.effective_angle = iac.estimate - correction;
        if (!std::isfinite(iac.effective_angle) ||
            std::abs(iac.effective_angle - adaptive->nominal_angle) > adaptive->max_angle_offset) {
          return finish("IAC1 effective angle exceeded its configured bound");
        }
        used_normal = iac1::normal(iac.effective_angle);
        const Vector acceleration = adaptive->kp * (iac.target - position) - adaptive->kv * velocity;
        if (!acceleration.allFinite() || acceleration.norm() > adaptive->max_acceleration) {
          return finish("IAC1 motion acceleration exceeded its configured bound");
        }
        const Vector motion_wrench = dynamics.inertia * acceleration;
        iac.estimator_rate = iac1::estimatorRate(adaptive->gamma, nominal_tangent, motion_wrench,
                                                rates.inertia_rate, velocity, used_normal,
                                                measured, target);
        wrench = dynamics.bias + motion_wrench;
        if (!std::isfinite(iac.estimator_rate)) {
          return finish("IAC1 estimator rate is invalid");
        }
      } else {
        iac.target = Vector(start_x, start_z);
        iac.effective_angle = adaptive->nominal_angle;
        iac.estimate = adaptive->initial_angle;
      }
      const Vector contact_wrench = -reference * used_normal;
      command_z = contact_wrench.y();
      wrench += contact_wrench;
      if (!wrench.allFinite() || wrench.norm() > adaptive->max_task_wrench) {
        return finish("IAC1 task wrench exceeded its configured bound");
      }
    } else {
      wrench.y() = command_z;
    }
    const Vector active_torque = task_jacobian.transpose() * wrench;

    Joints command{};
    bool active_limited = false;
    for (std::size_t j = 0; j < command.size(); ++j) {
      const bool active = j == joint2 || j == joint4;
      const double position_error = start[j] - state.q[j];
      const double excursion_limit = hybrid && j == joint4
          ? hybrid->joint4_excursion
          : (active ? settings.active_excursion : settings.held_excursion);
      if (!std::isfinite(position_error) || std::abs(position_error) > excursion_limit) {
        excursion = {j + 1, -position_error, excursion_limit};
        return finish("Joint excursion exceeded its limit");
      }
      double raw = settings.hold_kp[j] * position_error - settings.hold_kd[j] * state.dq[j];
      if (active) {
        raw = active_torque[j == joint2 ? 0 : 1];
      }
      const double previous = state.tau_J_d[j];
      if (!std::isfinite(raw) || std::abs(previous) > settings.torque_limit[j]) {
        return finish("Invalid torque or previous torque outside its limit");
      }
      const double magnitude_limited = std::clamp(raw, -settings.torque_limit[j], settings.torque_limit[j]);
      const double step = settings.torque_rate * std::min(dt, 0.001);
      command[j] = std::clamp(magnitude_limited, previous - step, previous + step);
      active_limited |= active && command[j] != raw;
    }

    samples.push_back(
        {elapsed, x, z, reference, external, integral_force, command_z, command, in_contact,
         desired_x, dx, x_acceleration, wrench.x(), wrench.y(), state.q, iac});
    if (!active_limited) {
      x_integral = next_x_integral;
    }
    const bool drives_out_of_force_saturation =
        (raw_force >= settings.max_force_command && error < 0.0) ||
        (raw_force <= 0.0 && error > 0.0);
    if (adaptive && in_contact && !active_limited) {
      iac.estimate += dt * iac.estimator_rate;
    }
    if (!adaptive && in_contact && !active_limited &&
        ((raw_force > 0.0 && raw_force < settings.max_force_command) ||
         drives_out_of_force_saturation)) {
      integral_force = std::clamp(integral_force + (gains.ki * dt) * error,
                                  -settings.integral_force_limit, settings.integral_force_limit);
    }

    // Franka supplies gravity and friction compensation.
    const franka::Torques torques(command);
    const bool complete = adaptive
        ? in_contact && elapsed - iac_start >= settings.duration
        : (hybrid ? unload_time >= settings.ramp_seconds : elapsed >= settings.duration);
    return complete ? franka::MotionFinished(torques) : torques;
  }, true);
  return reason;
}

void writeLog(std::ostream& log, const std::vector<Sample>& samples, double requested_force,
              const ExternalEstimate& baseline, CartesianForceMode mode) {
  const bool hybrid = mode == CartesianForceMode::hybrid;
  const bool adaptive = mode == CartesianForceMode::iac1;
  log << std::setprecision(17)
      << "elapsed_s,x_m,z_m,force_ref_N,external_fz_N,integral_force_N,command_fz_N";
  for (int j = 1; j <= 7; ++j) {
    log << ",tau_command_Nm_" << j;
  }
  log << ",contact_confirmed,force_target_N,external_fz_baseline_N,"
         "external_fx_N,external_fy_N,external_mx_Nm,external_my_Nm,external_mz_Nm";
  for (int j = 1; j <= 7; ++j) {
    log << ",tau_external_Nm_" << j << ",tau_external_baseline_Nm_" << j;
  }
  if (hybrid) {
    log << ",x_ref_m,dx_m_s,dx_ref_m_s,ddx_ref_m_s2,x_acceleration_command_m_s2,"
           "command_fx_N,total_command_fz_N";
  }
  if (adaptive) {
    log << ",x_ref_m,z_ref_m,normal_force_N,angle_estimate_rad,effective_angle_rad,"
           "angle_estimator_rate_rad_s,command_fx_N,total_command_fz_N";
  }
  if (hybrid || adaptive) {
    for (int joint = 1; joint <= 7; ++joint) {
      log << ",q_rad_" << joint;
    }
  }
  log << '\n';
  for (const auto& s : samples) {
    const auto& [fx, fy, fz, mx, my, mz] = s.external.wrench;
    log << s.time << ',' << s.x << ',' << s.z << ',' << s.reference << ',' << fz
        << ',' << s.integral_force << ',' << s.command_z;
    for (double torque : s.command) {
      log << ',' << torque;
    }
    log << ',' << (s.in_contact ? 1 : 0) << ',' << requested_force << ',' << baseline.wrench[2]
        << ',' << fx << ',' << fy << ',' << mx << ',' << my << ',' << mz;
    for (std::size_t j = 0; j < s.external.joint_torque.size(); ++j) {
      log << ',' << s.external.joint_torque[j] << ',' << baseline.joint_torque[j];
    }
    if (hybrid) {
      log << ',' << s.motion_reference.x << ',' << s.dx << ',' << s.motion_reference.dx
          << ',' << s.motion_reference.ddx << ',' << s.x_acceleration << ',' << s.command_x
          << ',' << s.total_command_z;
    }
    if (adaptive) {
      const auto& iac = s.adaptive;
      log << ',' << iac.target.x() << ',' << iac.target.y() << ',' << iac.normal_force
          << ',' << iac.estimate << ',' << iac.effective_angle << ',' << iac.estimator_rate
          << ',' << s.command_x << ',' << s.total_command_z;
    }
    if (hybrid || adaptive) {
      for (double angle : s.q) {
        log << ',' << angle;
      }
    }
    log << '\n';
  }
}

}  // namespace

int runCartesianForceProgram(int argc, char* argv[], CartesianForceMode mode) {
  const bool hybrid = mode == CartesianForceMode::hybrid;
  const bool adaptive = mode == CartesianForceMode::iac1;
  const bool help = argc == 2 && std::string_view(argv[1]) == "--help";
  const bool valid_arg_count = argc == (hybrid ? 3 : 2);
  if (help || !valid_arg_count) {
    std::cout << "Usage: " << argv[0] << (hybrid ? " FORCE_N DX_M\n" : " FORCE_N\n")
              << (adaptive ? "Regulate an x/z target with IAC1 using joints 2 and 4.\n"
                           : "Regulate downward force in base z using joints 2 and 4.\n")
              << "Enter h to return to the joint position captured at startup.\n"
              << "Enter t to move to the approach pose, then c to start force control.\n"
              << "Keep the tool free of contact at t so the force and torque estimates can be zeroed.\n"
              << "Place the surface within the configured no-contact travel limit.\n";
    if (hybrid) {
      std::cout << "DX_M is a signed one-way x displacement in meters.\n"
                << "Set the x move duration with x_motion_s in the config file.\n"
                << "The x trajectory and z-force ramp start together on c.\n"
                << "Settings: config/cartesian_hybrid_control.conf.\n";
    } else if (adaptive) {
      std::cout << "Use a low-friction surface. Settings: config/cartesian_iac1_control.conf.\n";
    } else {
      std::cout << "Settings: config/cartesian_contact.conf and config/cartesian_force_control.conf.\n";
    }
    return help ? 0 : 2;
  }

  const std::filesystem::path project(FRANKA_PROJECT_DIR);
  const std::string name = hybrid ? "cartesian_hybrid_control"
      : (adaptive ? "cartesian_iac1_control" : "cartesian_force_control");
  ContactSettings settings;
  HybridMotion motion;
  Iac1Settings iac;
  Gains gains;
  try {
    const auto config_path = project / "config" / (name + ".conf");
    std::map<std::string, double*> keys{{"contact_loss_N", &settings.contact_loss}};
    if (!adaptive) {
      keys.insert({{"force_kp", &gains.kp}, {"force_ki", &gains.ki}});
    }
    if (hybrid) {
      keys.insert({{"x_kp", &motion.kp}, {"x_kd", &motion.kd}, {"x_ki", &motion.ki},
                   {"x_max_acceleration", &motion.max_acceleration},
                   {"joint_4_excursion_rad", &motion.joint4_excursion},
                   {"x_max_offset_m", &motion.max_offset}, {"x_motion_s", &motion.motion_seconds},
                   {"x_final_hold_s", &motion.hold_seconds}, {"x_max_error_m", &motion.max_error},
                   {"x_max_speed_m_s", &motion.max_speed},
                   {"x_integral_limit_m_s2", &motion.max_integral},
                   {"min_xz_singular_value_m_rad", &motion.min_singular_value},
                   {"jacobian_filter_time_s", &motion.jacobian_filter_time}});
      loadContactSettings(config_path, settings, keys,
                          {"force_kp", "force_ki", "x_kp", "x_kd", "x_ki", "x_final_hold_s"});
    } else if (adaptive) {
      keys.insert({
          {"run_duration_s", &settings.duration}, {"target_x_offset_m", &iac.target_offset.x()},
          {"target_z_offset_m", &iac.target_offset.y()}, {"nominal_angle_rad", &iac.nominal_angle},
          {"initial_effective_angle_rad", &iac.initial_angle},
          {"max_angle_offset_rad", &iac.max_angle_offset}, {"adaptation_gamma", &iac.gamma},
          {"motion_kp", &iac.kp}, {"motion_kv", &iac.kv},
          {"max_motion_acceleration_m_s2", &iac.max_acceleration},
          {"model_difference_step_s", &iac.difference_step},
          {"min_xz_singular_value_m_rad", &iac.min_singular_value},
          {"max_x_speed_m_s", &iac.max_x_speed}, {"max_target_offset_m", &iac.max_target_offset},
          {"max_task_wrench_N", &iac.max_task_wrench},
      });
      loadContactSettings(config_path, settings, keys,
                          {"target_x_offset_m", "target_z_offset_m",
                           "nominal_angle_rad", "initial_effective_angle_rad"}, false);
      if (std::abs(iac.nominal_angle) + iac.max_angle_offset >= std::acos(0.0) ||
          std::abs(iac.initial_angle - iac.nominal_angle) > iac.max_angle_offset ||
          iac.target_offset.norm() > std::min(iac.max_target_offset, settings.max_tool_travel) ||
          settings.max_target > iac.max_task_wrench) {
        throw std::runtime_error("IAC1 angle, target offset, or wrench bounds are inconsistent");
      }
    } else {
      loadContactSettings(project / "config" / "cartesian_contact.conf", settings);
      keys.emplace("run_duration_s", &settings.duration);
      readGainConfig(config_path, keys, {"force_kp", "force_ki"});
    }
    if (!validJointSettings(settings) ||
        (settings.ignore_realtime != 0.0 && settings.ignore_realtime != 1.0) ||
        settings.min_target > settings.max_target ||
        settings.max_target > settings.max_force_command ||
        settings.max_target > settings.max_measured ||
        settings.contact_loss > settings.min_contact ||
        settings.min_contact > settings.max_measured ||
        settings.contact_confirmation > settings.contact_timeout ||
        settings.no_contact_travel > settings.max_tool_travel ||
        (!hybrid && !adaptive && settings.duration < 2.0 * settings.ramp_seconds) ||
        gains.kp < 0.0 || gains.ki < 0.0 || motion.kp < 0.0 || motion.kd < 0.0 ||
        motion.ki < 0.0 || motion.hold_seconds < 0.0) {
      throw std::runtime_error("Inconsistent contact limits, timing, or gains");
    }
  } catch (const std::exception& error) {
    std::cerr << "Invalid controller config: " << error.what() << '\n';
    return 1;
  }

  double target = 0.0;
  try {
    const std::string input(argv[1]);
    std::size_t end = 0;
    target = std::stod(input, &end);
    if (end != input.size() || !std::isfinite(target) || target < settings.min_target ||
        target > settings.max_target) {
      throw std::invalid_argument("Force target must be between " + std::to_string(settings.min_target) +
                                  " and " + std::to_string(settings.max_target) + " N");
    }
    if (hybrid) {
      const std::string offset(argv[2]);
      motion.offset = std::stod(offset, &end);
      if (end != offset.size() || !std::isfinite(motion.offset) ||
          std::abs(motion.offset) > motion.max_offset) {
        throw std::invalid_argument("x displacement must be within +/-" +
                                    std::to_string(motion.max_offset) + " m");
      }
      // The normalized quintic profile reaches its peak speed at the midpoint.
      const double minimum_seconds = 1.875 * std::abs(motion.offset) / motion.max_speed;
      if (motion.motion_seconds < minimum_seconds) {
        throw std::invalid_argument("x duration is too short for the configured x-speed limit "
                                    "(minimum " + std::to_string(minimum_seconds) + " s)");
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "Invalid control argument: " << error.what() << '\n';
    return 2;
  }

  try {
    if (std::signal(SIGINT, requestStop) == SIG_ERR) {
      throw std::runtime_error("Could not register Ctrl+C handler");
    }
    std::vector<Sample> samples;
    const double max_duration = hybrid
        ? motion.motion_seconds + motion.hold_seconds + settings.ramp_seconds
        : settings.duration + (adaptive ? settings.contact_timeout : 0.0);
    const double capacity = std::ceil(max_duration * 1000.0) + 100.0;
    if (capacity >= static_cast<double>(samples.max_size())) {
      throw std::runtime_error("Configured duration exceeds the log capacity");
    }
    samples.reserve(static_cast<std::size_t>(capacity));
    franka::Robot robot(settings.robot_host, settings.ignore_realtime == 1.0
                                                ? franka::RealtimeConfig::kIgnore
                                                : franka::RealtimeConfig::kEnforce);
    const Joints startup_pose = robot.readOnce().q;
    ExternalEstimate baseline{};
    bool pose_reached = false;
    while (!interrupted) {
      std::cout << "h: startup pose | t: approach pose | c: force control | q: quit (press Enter to submit)\n"
                << std::flush;
      const char key = readCommand();
      if (key == 'q' || interrupted) {
        return 0;
      }
      if (key == 'h') {
        pose_reached = false;
        joint_trajectory::move(robot, startup_pose, &interrupted, settings.joint_move);
        std::cout << "Return trajectory completed.\n";
        continue;
      }
      if (key == 't') {
        joint_trajectory::move(robot, settings.contact_pose, &interrupted, settings.joint_move);
        if (interrupted) {
          return 0;
        }
        const auto state = robot.readOnce();
        pose_reached = atContactPose(state.q, settings);
        if (!pose_reached) {
          std::cerr << "Arm did not settle within " << settings.pose_tolerance
                    << " rad of the joint pose.\n";
          continue;
        }
        baseline = externalEstimate(state);
        pose_reached = baseline.isFinite();
        if (!pose_reached) {
          std::cerr << "Could not zero a non-finite external wrench or torque estimate.\n";
          continue;
        }
        std::cout << "Joint pose reached. External wrench and joint torque zeroed. "
                  << "Captured z-force offset: " << baseline.wrench[2] << " N.\n";
      }
      if (key == 'c') {
        if (!pose_reached) {
          std::cerr << "Move to the joint pose with t first.\n";
          continue;
        }
        const auto state = robot.readOnce();
        pose_reached = atContactPose(state.q, settings);
        const auto external = zeroedEstimate(externalEstimate(state), baseline);
        const double force = adaptive ? std::hypot(external.wrench[0], external.wrench[2])
                                      : std::abs(external.wrench[2]);
        if (!pose_reached || !external.isFinite() || force > settings.max_measured) {
          std::cerr << "Force control requires the preset pose and an estimated force within "
                    << settings.max_measured << " N.\n";
          continue;
        }
        pose_reached = false;
        samples.clear();
        const auto model = robot.loadModel();
        const auto path = reserveLogPath(project / "data" / name);
        std::ofstream log(path);
        if (!log) {
          throw std::runtime_error("Could not open " + path.string());
        }

        bool control_failed = false;
        JointExcursion excursion;
        try {
          if (const char* reason = run(robot, model, gains, settings, target, baseline, samples,
                                       hybrid ? &motion : nullptr, adaptive ? &iac : nullptr,
                                       excursion)) {
            std::cerr << "Controller stopped: " << reason;
            if (excursion.joint != 0) {
              std::cerr << " (joint " << excursion.joint << ", displacement "
                        << excursion.displacement << " rad, limit ±" << excursion.limit << " rad)";
            }
            std::cerr << '\n';
          }
        } catch (const std::exception& error) {
          std::cerr << "Force control failed: " << error.what() << '\n';
          control_failed = true;
        }
        writeLog(log, samples, target, baseline, mode);
        log.close();
        if (!log) {
          throw std::runtime_error("Could not save complete CSV to " + path.string());
        }
        std::cout << "Saved " << samples.size() << " samples to " << path << '\n';
        if (control_failed || interrupted) {
          return 1;
        }
      }
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Cartesian force control failed: " << error.what() << '\n';
    return 1;
  }
}
