// Copyright 2022 Yifeng Zhu

#include "franka_controller.pb.h"
#include "franka_robot_state.pb.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "utils/common_utils.h"
#include "utils/control_utils.h"
#include "utils/robot_utils.h"

#include "controllers/joint_impedance.h"

#include <memory>

namespace controller {
JointImpedanceController::JointImpedanceController() {}
JointImpedanceController::~JointImpedanceController() {}

JointImpedanceController::JointImpedanceController(franka::Model &model) {
  model_ = &model;
}

bool JointImpedanceController::ParseMessage(const FrankaControlMessage &msg) {

  if (!msg.control_msg().UnpackTo(&control_msg_)) {
    return false;
  }

  // Kp << control_msg_.kp();
  // Kd << control_msg_.kd();

  std::vector<double> kp_array;
  std::vector<double> kd_array;

  kp_array.reserve(control_msg_.kp().size());
  kd_array.reserve(control_msg_.kd().size());

  for (double kp_i : control_msg_.kp()) {
    kp_array.push_back(kp_i);
  }
  for (double kd_i : control_msg_.kd()) {
    kd_array.push_back(kd_i);
  }

  Kp << Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kp_array.data());
  Kd << Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kd_array.data());

  joint_max_ << 2.8978, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
  joint_min_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  // Computed-torque feedforward (optional). Absent field or malformed arrays
  // degrade to the exact baseline law rather than injecting garbage torque.
  ff_enable_ = false;
  ff_vel_scale_ = 0.;
  ff_acc_scale_ = 0.;
  use_coriolis_ = false;
  if (control_msg_.has_feedforward()) {
    const auto &ff = control_msg_.feedforward();
    if (ff.ff_enable() && ff.dq_d_size() == 7 && ff.ddq_d_size() == 7) {
      ff_enable_ = true;
      ff_vel_scale_ = ff.ff_vel_scale();
      ff_acc_scale_ = ff.ff_acc_scale();
      use_coriolis_ = ff.use_coriolis();
    } else if (ff.ff_enable()) {
      // Feedforward was requested but dq_d/ddq_d are not both length 7. We fall
      // back to the baseline law rather than inject garbage torque -- but say so
      // loudly (once), so an operator who asked for FF and silently got baseline
      // isn't left guessing. Warn once to avoid spamming the 20 Hz message loop.
      static bool warned = false;
      if (!warned) {
        std::cerr << "[JointImpedanceController] feedforward requested but dq_d/"
                     "ddq_d are not both length 7 (got "
                  << ff.dq_d_size() << "/" << ff.ddq_d_size()
                  << "); running baseline JOINT_IMPEDANCE." << std::endl;
        warned = true;
      }
    }
  }

  this->state_estimator_ptr_->ParseMessage(msg.state_estimator_msg());
  return true;
}

void JointImpedanceController::ComputeGoal(
    const std::shared_ptr<StateInfo> &current_state_info,
    std::shared_ptr<StateInfo> &goal_state_info) {
  if (control_msg_.goal().is_delta()) {
    Eigen::Matrix<double, 7, 1> delta_joint_position;
    delta_joint_position << control_msg_.goal().q1(), control_msg_.goal().q2(),
        control_msg_.goal().q3(), control_msg_.goal().q4(),
        control_msg_.goal().q5(), control_msg_.goal().q6(),
        control_msg_.goal().q7();
    goal_state_info->joint_positions =
        current_state_info->joint_positions + delta_joint_position;
  } else {
    goal_state_info->joint_positions << control_msg_.goal().q1(),
        control_msg_.goal().q2(), control_msg_.goal().q3(),
        control_msg_.goal().q4(), control_msg_.goal().q5(),
        control_msg_.goal().q6(), control_msg_.goal().q7();
  }
  // goal_state_info->joint_positions << control_msg_.goal().q1(),
  // control_msg_.goal().q2(), control_msg_.goal().q3(),
  // control_msg_.goal().q4(), control_msg_.goal().q5(),
  // control_msg_.goal().q6(), control_msg_.goal().q7();

  // Desired joint velocity / acceleration for the feedforward path. Absolute
  // even when the position goal is a delta. Zeroed unless feedforward is on.
  goal_state_info->joint_velocities.setZero();
  goal_state_info->joint_accelerations.setZero();
  if (ff_enable_) {
    for (int i = 0; i < 7; i++) {
      goal_state_info->joint_velocities[i] = control_msg_.feedforward().dq_d(i);
      goal_state_info->joint_accelerations[i] =
          control_msg_.feedforward().ddq_d(i);
    }
  }
}

std::array<double, 7>
JointImpedanceController::Step(const franka::RobotState &robot_state,
                               const Eigen::Matrix<double, 7, 1> &desired_q) {
  // Thin wrapper for the baseline (feedforward-off) path.
  Eigen::Matrix<double, 7, 1> zero = Eigen::Matrix<double, 7, 1>::Zero();
  return Step(robot_state, desired_q, zero, zero);
}

std::array<double, 7>
JointImpedanceController::Step(const franka::RobotState &robot_state,
                               const Eigen::Matrix<double, 7, 1> &desired_q,
                               const Eigen::Matrix<double, 7, 1> &desired_dq,
                               const Eigen::Matrix<double, 7, 1> &desired_ddq) {

  std::chrono::high_resolution_clock::time_point t1 =
      std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 7, 1> tau_d;

  std::array<double, 49> mass_array = model_->mass(robot_state);
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());

  // coriolis and gravity
  std::array<double, 7> coriolis_array = model_->coriolis(robot_state);
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // Current joint velocity
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // Current joint position
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

  Eigen::MatrixXd joint_pos_error(7, 1);

  Eigen::Matrix<double, 7, 1> current_q, current_dq;

  if (this->state_estimator_ptr_->IsFirstState()) {
    this->state_estimator_ptr_->Initialize(q, dq);
  } else {
    this->state_estimator_ptr_->Update(q, dq);
  }

  // current_q_ and current_dq_ will be raw data if estimation flag is set to
  // false
  current_q = this->state_estimator_ptr_->GetCurrentJointPos();
  current_dq = this->state_estimator_ptr_->GetCurrentJointVel();
  joint_pos_error << desired_q - current_q;

  tau_d << Kp.cwiseProduct(joint_pos_error) - Kd.cwiseProduct(current_dq);
  if (ff_enable_) {
    // Velocity feedforward collapses the cruise lag; M * ddq_d supplies the
    // acceleration torque from the budget. Gravity stays libfranka-implicit
    // (no gravity term added here).
    tau_d += Kd.cwiseProduct(ff_vel_scale_ * desired_dq) +
             M * (ff_acc_scale_ * desired_ddq);
    if (use_coriolis_)
      tau_d += coriolis;
  }
  // joint_pos_error << desired_q_ - q;
  // tau_d << Kp.cwiseProduct(joint_pos_error) - Kd.cwiseProduct(dq);

  Eigen::Matrix<double, 7, 1> dist2joint_max;
  Eigen::Matrix<double, 7, 1> dist2joint_min;

  dist2joint_max = joint_max_.matrix() - current_q;
  dist2joint_min = current_q - joint_min_.matrix();

  for (int i = 0; i < 7; i++) {
    if (dist2joint_max[i] < 0.1 && tau_d[i] > 0.)
      tau_d[i] = 0.;
    if (dist2joint_min[i] < 0.1 && tau_d[i] < 0.)
      tau_d[i] = 0.;
  }

  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

  return tau_d_array;
}
} // namespace controller
