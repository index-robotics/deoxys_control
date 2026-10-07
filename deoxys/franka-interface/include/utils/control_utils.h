// Copyright 2022 Yifeng Zhu

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_CONTROL_UTILS_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_CONTROL_UTILS_H_

namespace control_utils {

// void PInverse(const Eige::MatrixXd& M, Eigen::Matrix& M_inv, double
// epsilon=0.00025);
inline void PInverse(const Eigen::MatrixXd &M, Eigen::MatrixXd &M_inv,
                     double epsilon = 0.00025) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singular_vals =
      svd.singularValues();

  Eigen::MatrixXd S_inv = M;
  S_inv.setZero();
  for (int i = 0; i < singular_vals.size(); i++) {
    if (singular_vals(i) < epsilon) {
      S_inv(i, i) = 0.;
    } else {
      S_inv(i, i) = 1. / singular_vals(i);
    }
  }
  M_inv = Eigen::MatrixXd(svd.matrixV() * S_inv * svd.matrixU().transpose());
}

inline void CartesianVelocitySafetyGuardFn(std::array<double, 6> &twist_array,
                                  double min_twist_trans, 
                                  double max_twist_trans, 
                                  double min_twist_rot, 
                                  double max_twist_rot) {
  for (size_t i = 0; i < 3; i++) {
    if (twist_array[i] < min_twist_trans) {
      twist_array[i] = min_twist_trans;
    } else if (twist_array[i] > max_twist_trans) {
      twist_array[i] = max_twist_trans;
    }
  }

  for (size_t i = 3; i < 6; i++) {
    if (twist_array[i] < min_twist_rot) {
      twist_array[i] = min_twist_rot;
    } else if (twist_array[i] > max_twist_rot) {
      twist_array[i] = max_twist_rot;
    }
  }
}

inline void JointVelocitySafetyGuardFn(std::array<double, 7> &dq_array,
                                  double min_dq, double max_dq) {
  for (size_t i = 0; i < dq_array.size(); i++) {
    if (dq_array[i] < min_dq) {
      dq_array[i] = min_dq;
    } else if (dq_array[i] > max_dq) {
      dq_array[i] = max_dq;
    }
  }
}

inline void TorqueSafetyGuardFn(std::array<double, 7> &tau_d_array,
                                const std::array<double, 7> &min_torque,
                                const std::array<double, 7> &max_torque) {
  for (size_t i = 0; i < tau_d_array.size(); i++) {
    if (tau_d_array[i] < min_torque[i]) {
      tau_d_array[i] = min_torque[i];
    } else if (tau_d_array[i] > max_torque[i]) {
      tau_d_array[i] = max_torque[i];
    }
  }
}

// Axis-angle error (rad) of current vs desired, rotated by -R_EE_in_base,
// shrunk by `deadband` (no torque step at its edge) and clipped to `max_norm`.
inline Eigen::Vector3d OrientationError(const Eigen::Quaterniond &desired,
                                        const Eigen::Quaterniond &current,
                                        const Eigen::Matrix3d &R_EE_in_base,
                                        double deadband = 1e-2,
                                        double max_norm = 0.5) {
  Eigen::Quaterniond q_err(desired.inverse() * current);
  if (q_err.w() < 0.0) {
    q_err.coeffs() << -q_err.coeffs();
  }
  const double s = q_err.vec().norm();
  // Exact log map; 2*vec(q) is its small-angle limit.
  Eigen::Vector3d err = (s > 1e-9)
                            ? Eigen::Vector3d(2.0 * std::atan2(s, q_err.w()) /
                                              s * q_err.vec())
                            : Eigen::Vector3d(2.0 * q_err.vec());
  err = -R_EE_in_base * err;
  const double n = err.norm();
  if (n <= deadband) {
    return Eigen::Vector3d::Zero();
  }
  return err * (std::min(n, max_norm + deadband) - deadband) / n;
}

} // namespace control_utils

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_CONTROL_UTILS_H_
