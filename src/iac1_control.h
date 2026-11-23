#pragma once

#include "planar_dynamics.h"

#include <franka/robot_state.h>

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

inline double estimatorRate(double gamma, const Vector& nominal_tangent,
                            const Vector& motion_wrench, const Matrix& inertia_rate,
                            const Vector& velocity, const Vector& effective_normal,
                            double measured_force, double desired_force) {
  return gamma * nominal_tangent.dot(
      motion_wrench + inertia_rate * velocity + effective_normal * (measured_force - desired_force));
}

}  // namespace iac1
