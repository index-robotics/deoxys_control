// OSC_POSE task torque for desired linear / angular task accelerations.

#include <Eigen/Dense>

#include "utils/control_utils.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_TASK_TORQUE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_TASK_TORQUE_H_

namespace controller {

// full: J^T (J M^-1 J^T)^-1 [a_p; a_r], so the end effector accelerates as asked.
// Decoupled (the OSC_POSE default): each block's inertia on its own,
// J_p^T (J_p M^-1 J_p^T)^-1 a_p + J_r^T (J_r M^-1 J_r^T)^-1 a_r, which drops the
// translation/rotation cross terms: a pure rotation also accelerates the EE
// linearly, more so the further the EE sits from the wrist.
inline Eigen::Matrix<double, 7, 1>
OscTaskTorque(const Eigen::Matrix<double, 6, 7> &jacobian,
              const Eigen::Matrix<double, 7, 7> &M_inv,
              const Eigen::Vector3d &a_p, const Eigen::Vector3d &a_r,
              bool full) {
  if (full) {
    Eigen::MatrixXd Lambda_inv = jacobian * M_inv * jacobian.transpose();
    Eigen::MatrixXd Lambda;
    control_utils::PInverse(Lambda_inv, Lambda);
    Eigen::Matrix<double, 6, 1> a;
    a << a_p, a_r;
    return jacobian.transpose() * (Lambda * a);
  }
  const Eigen::Matrix<double, 3, 7> J_p = jacobian.topRows<3>();
  const Eigen::Matrix<double, 3, 7> J_r = jacobian.bottomRows<3>();
  Eigen::MatrixXd L_p_inv = J_p * M_inv * J_p.transpose();
  Eigen::MatrixXd L_r_inv = J_r * M_inv * J_r.transpose();
  Eigen::MatrixXd L_p, L_r;
  control_utils::PInverse(L_p_inv, L_p);
  control_utils::PInverse(L_r_inv, L_r);
  return J_p.transpose() * (L_p * a_p) + J_r.transpose() * (L_r * a_r);
}

} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_TASK_TORQUE_H_
