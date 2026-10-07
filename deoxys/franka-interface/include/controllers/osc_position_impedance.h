// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "controllers/base_controller.h"

#ifndef CONTROLLERS_OSC_POSITION_IMPEDANCE_H_
#define CONTROLLERS_OSC_POSITION_IMPEDANCE_H_

namespace controller {
class OSCPositionImpedanceController : public BaseController {
protected:
  FrankaOSCPoseControllerMessage control_msg_;
  // Zero-initialized: ParseMessage writes only the diagonals, and Eigen leaves
  // fixed-size matrices uninitialized, so otherwise the off-diagonal gains are
  // whatever memory the controller was allocated on.
  Eigen::Matrix<double, 3, 3> Kp_p = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> Kp_r = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> Kd_p = Eigen::Matrix<double, 3, 3>::Zero();
  Eigen::Matrix<double, 3, 3> Kd_r = Eigen::Matrix<double, 3, 3>::Zero();

  Eigen::Matrix<double, 7, 1> static_q_task_;

  Eigen::Vector3d prev_goal_pos_EE_in_base_frame;

  Eigen::Matrix<double, 7, 1> residual_mass_vec_;

public:
  OSCPositionImpedanceController();
  OSCPositionImpedanceController(franka::Model &model);

  ~OSCPositionImpedanceController();

  bool ParseMessage(const FrankaControlMessage &msg);

  // void ComputeGoal(const Eigen::Vector3d&, const Eigen::Quaterniond&,
  // Eigen::Vector3d&, Eigen::Quaterniond&);
  void ComputeGoal(const std::shared_ptr<StateInfo> &,
                   std::shared_ptr<StateInfo> &);

  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Vector3d &,
                             const Eigen::Quaterniond &);
};
} // namespace controller

#endif
