#pragma once

#include <franka/model.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <array>

namespace planar_dynamics {

using Vector = Eigen::Vector2d;
using Matrix = Eigen::Matrix2d;
using PlanarJacobian = Eigen::Matrix<double, 2, 7>;
inline constexpr std::array<int, 2> active{1, 3};
inline constexpr double min_singular_value = 0.05;  // m/rad

inline PlanarJacobian planarJacobian(const std::array<double, 42>& values) {
  const Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(values.data());
  PlanarJacobian planar;
  planar.row(0) = jacobian.row(0);
  planar.row(1) = jacobian.row(2);
  return planar;
}

inline Matrix activeJacobian(const PlanarJacobian& planar) {
  Matrix jacobian;
  jacobian.col(0) = planar.col(active[0]);
  jacobian.col(1) = planar.col(active[1]);
  return jacobian;
}

inline bool usable(const Matrix& jacobian, double threshold = min_singular_value) {
  return jacobian.allFinite() && jacobian.jacobiSvd().singularValues()[1] >= threshold;
}

struct Dynamics {
  Matrix inertia;
  Vector bias;
};

inline Dynamics cartesianDynamics(const Matrix& jacobian, const Matrix& mass,
                           const Vector& coriolis, const Vector& jacobian_bias) {
  const auto solve_transpose = jacobian.transpose().partialPivLu();
  const Matrix mass_times_inverse = solve_transpose.solve(mass.transpose()).transpose();
  const Matrix inertia = solve_transpose.solve(mass_times_inverse);
  return {inertia, solve_transpose.solve(coriolis) - inertia * jacobian_bias};
}

inline Matrix activeMass(const franka::Model& model, const franka::RobotState& state) {
  const auto mass_values = model.mass(state);
  const Eigen::Map<const Eigen::Matrix<double, 7, 7>> full_mass(mass_values.data());
  Matrix mass;
  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      mass(row, col) = full_mass(active[row], active[col]);
    }
  }

  return mass;
}

inline Dynamics cartesianDynamics(const franka::Model& model, const franka::RobotState& state,
                                  const Matrix& jacobian, const Vector& jacobian_bias) {
  const auto coriolis = model.coriolis(state);
  return cartesianDynamics(jacobian, activeMass(model, state),
                           Vector(coriolis[active[0]], coriolis[active[1]]), jacobian_bias);
}

inline void updateJacobianDerivative(PlanarJacobian& derivative, const PlanarJacobian& current,
                                     const PlanarJacobian& previous, double dt,
                                     double filter_time = 0.01) {
  derivative += (current - previous - dt * derivative) / (filter_time + dt);
}

}  // namespace planar_dynamics
