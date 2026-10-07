// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_H_

namespace controller {
class OSCImpedanceController : public BaseController {
protected:
  FrankaOSCPoseControllerMessage control_msg_;
  // Zero-initialized: ParseMessage writes only the diagonals, and Eigen leaves
  // fixed-size matrices uninitialized, so otherwise the off-diagonal gains are
  // whatever memory the controller was allocated on.
  Eigen::Matrix<double, 3, 3> Kp_p = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> Kp_r = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> Kd_p = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> Kd_r = Eigen::Matrix<double, 3, 3>::Zero();

  Eigen::Matrix<double, 7, 1> residual_mass_vec_;

  Eigen::Matrix<double, 7, 1> static_q_task_;
  Eigen::Array<double, 7, 1> joint_max_;
  Eigen::Array<double, 7, 1> joint_min_;
  Eigen::Array<double, 7, 1> avoidance_weights_;

  // Feedforward gate + scales (default off => baseline law).
  bool ff_enable_ = false;
  double ff_vel_scale_ = 0.;
  double ff_acc_scale_ = 0.;

public:
  OSCImpedanceController();
  OSCImpedanceController(franka::Model &model);

  ~OSCImpedanceController();

  bool ParseMessage(const FrankaControlMessage &msg);

  void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                   std::shared_ptr<StateInfo> &goal_info);

  // Thin 3-arg Step delegates to the 7-arg one with zero feedforward.
  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Vector3d &,
                             const Eigen::Quaterniond &);
  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Vector3d &desired_pos,
                             const Eigen::Quaterniond &desired_quat,
                             const Eigen::Vector3d &desired_v,
                             const Eigen::Vector3d &desired_w,
                             const Eigen::Vector3d &desired_a,
                             const Eigen::Vector3d &desired_alpha);
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_OSC_IMPEDANCE_H_
