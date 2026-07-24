// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>

#include "controllers/base_controller.h"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_IMPEDANCE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_IMPEDANCE_H_

namespace controller {
class JointImpedanceController : public BaseController {
protected:
  FrankaJointImpedanceControllerMessage control_msg_;
  Eigen::Matrix<double, 7, 1> Kp, Kd;

  Eigen::Matrix<double, 7, 1> static_q_task_;
  Eigen::Array<double, 7, 1> joint_max_;
  Eigen::Array<double, 7, 1> joint_min_;

  // Eigen::Matrix<double, 7, 1> smooth_current_q_;
  // Eigen::Matrix<double, 7, 1> smooth_prev_q_;
  // Eigen::Matrix<double, 7, 1> smooth_current_dq_;

  // double alpha_q_;
  // double alpha_dq_;

  bool first_state_ = true;

  // Computed-torque feedforward gates + scales (default off => baseline law).
  bool ff_enable_ = false;
  double ff_vel_scale_ = 0.;
  double ff_acc_scale_ = 0.;
  bool use_coriolis_ = false;

public:
  JointImpedanceController();
  JointImpedanceController(franka::Model &model);

  ~JointImpedanceController();

  bool ParseMessage(const FrankaControlMessage &msg);

  void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                   std::shared_ptr<StateInfo> &goal_info);

  // Thin 2-arg Step delegates to the 4-arg one with zero feedforward.
  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Matrix<double, 7, 1> &);
  std::array<double, 7> Step(const franka::RobotState &,
                             const Eigen::Matrix<double, 7, 1> &desired_q,
                             const Eigen::Matrix<double, 7, 1> &desired_dq,
                             const Eigen::Matrix<double, 7, 1> &desired_ddq);
};
} // namespace controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_JOINT_IMPEDANCE_H_
