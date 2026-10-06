// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_STATE_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_STATE_H_

struct StateInfo {
  Eigen::Vector3d pos_EE_in_base_frame;
  Eigen::Quaterniond quat_EE_in_base_frame;
  Eigen::Matrix<double, 7, 1> joint_positions;
  // Desired joint velocity / acceleration at the goal, used by the
  // computed-torque feedforward path. Zero-initialized because the extended
  // joint Reset reads them for every joint-interpolator controller, and the
  // enclosing struct is otherwise a plain aggregate with uninitialized Eigen
  // members.
  Eigen::Matrix<double, 7, 1> joint_velocities =
      Eigen::Matrix<double, 7, 1>::Zero();
  Eigen::Matrix<double, 7, 1> joint_accelerations =
      Eigen::Matrix<double, 7, 1>::Zero();
  // EE twist; on the goal, also the OSC_POSE velocity feedforward. Desired EE
  // acceleration at the goal for the OSC_POSE acceleration feedforward.
  // Zero-initialized because the extended pose Reset reads them for every
  // pose-interpolator controller.
  Eigen::Vector3d twist_trans_EE_in_base_frame = Eigen::Vector3d::Zero();
  Eigen::Vector3d twist_rot_EE_in_base_frame = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_trans_EE_in_base_frame = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_rot_EE_in_base_frame = Eigen::Vector3d::Zero();
};
#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_SHARED_STATE_H_
