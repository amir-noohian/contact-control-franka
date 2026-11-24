#pragma once

#include "planar_dynamics.h"

#include <franka/robot_state.h>

#include <algorithm>
#include <cmath>

namespace iac1 {

using planar_dynamics::Matrix;
using planar_dynamics::Vector;

inline Vector tangent(double angle) {
  return {std::cos(angle), std::sin(angle)};
}

inline Vector normal(double angle) {
  return {-std::sin(angle), std::cos(angle)};
}

struct AngleStability {
  double start = -1.0;
  double minimum = 0.0;
  double maximum = 0.0;

  void observe(double angle, double elapsed, double tolerance) {
    if (start < 0.0 || std::max(maximum, angle) - std::min(minimum, angle) > tolerance) {
      start = elapsed;
      minimum = maximum = angle;
    } else {
      minimum = std::min(minimum, angle);
      maximum = std::max(maximum, angle);
    }
  }

  double stableSeconds(double elapsed) const {
    return start < 0.0 ? 0.0 : elapsed - start;
  }
};

struct Reference {
  Vector position;
  Vector velocity;
  Vector acceleration;
};

inline Reference trajectory(const Vector& start, const Vector& offset,
                            double elapsed, double duration) {
  const double s = std::clamp(elapsed / duration, 0.0, 1.0);
  const double blend = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
  const double speed = 30.0 * s * s * (1.0 - s) * (1.0 - s) / duration;
  const double acceleration = 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s) / (duration * duration);
  return {start + blend * offset, speed * offset, acceleration * offset};
}

struct ModelRates {
  Vector jacobian_bias;
  Matrix inertia_rate;
};

// Differentiate the model along the active joint velocities, with the held joints fixed.
inline bool modelRates(const franka::Model& model, const franka::RobotState& state,
                       const Vector& velocity, double step, double min_singular_value,
                       ModelRates& rates) {
  using namespace planar_dynamics;
  auto before = state;
  auto after = state;
  for (int i = 0; i < 2; ++i) {
    const int joint = active[i];
    before.q[joint] -= step * velocity[i];
    after.q[joint] += step * velocity[i];
  }
  const auto jacobianAt = [&](const franka::RobotState& sample) {
    return activeJacobian(planarJacobian(model.zeroJacobian(franka::Frame::kEndEffector, sample)));
  };
  const Matrix j_before = jacobianAt(before);
  const Matrix j_after = jacobianAt(after);
  if (!usable(j_before, min_singular_value) || !usable(j_after, min_singular_value)) {
    return false;
  }
  const Matrix lambda_before =
      cartesianDynamics(j_before, activeMass(model, before), Vector::Zero(), Vector::Zero()).inertia;
  const Matrix lambda_after =
      cartesianDynamics(j_after, activeMass(model, after), Vector::Zero(), Vector::Zero()).inertia;
  rates.jacobian_bias = (j_after - j_before) * velocity / (2.0 * step);
  rates.inertia_rate = (lambda_after - lambda_before) / (2.0 * step);
  return rates.jacobian_bias.allFinite() && rates.inertia_rate.allFinite();
}

inline double momentumCorrection(double gamma, const Vector& nominal_tangent,
                                 const Matrix& inertia, const Vector& velocity) {
  return gamma * nominal_tangent.dot(inertia * velocity);
}

inline Vector xMotionWrench(const planar_dynamics::Dynamics& dynamics, double x_acceleration) {
  return {dynamics.inertia(0, 0) * x_acceleration + dynamics.bias.x(), 0.0};
}

inline Vector tangentMotionWrench(const planar_dynamics::Dynamics& dynamics,
                                  const Vector& tangent, double acceleration) {
  return tangent * (tangent.dot(dynamics.inertia * tangent) * acceleration +
                    tangent.dot(dynamics.bias));
}

inline double estimatorRate(double gamma, const Vector& nominal_tangent,
                            const Vector& motion_wrench, const Matrix& inertia_rate,
                            const Vector& velocity, const Vector& effective_normal,
                            double measured_force, double commanded_compression) {
  return gamma * nominal_tangent.dot(
      motion_wrench + inertia_rate * velocity +
      effective_normal * (measured_force - commanded_compression));
}

}  // namespace iac1
