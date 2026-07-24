// Copyright 2022 Yifeng Zhu

#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "franka_controller.pb.h"
#include "utils/shared_state.h"
#include "utils/state_estimators/base_state_estimator.h"
#include "utils/traj_interpolators/base_traj_interpolator.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreturn-type"

#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_BASE_CONTROLLER_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_BASE_CONTROLLER_H_

namespace controller {
class BaseController {
protected:
  std::shared_ptr<estimator_utils::BaseStateEstimator> state_estimator_ptr_;

public:
  franka::Model *model_;

  inline BaseController(){};

  inline ~BaseController(){};

  inline virtual bool ParseMessage(const FrankaControlMessage &msg){};

  inline virtual void ComputeGoal(const std::shared_ptr<StateInfo> &state_info,
                                  std::shared_ptr<StateInfo> &goal_info){};

  // For pose
  inline virtual std::array<double, 7> Step(const franka::RobotState &,
                                            const Eigen::Vector3d &,
                                            const Eigen::Quaterniond &){};

  // For cartesian velocity
  inline virtual std::array<double, 6> Step(const franka::RobotState &,
                                            const Eigen::Vector3d &,
                                            const Eigen::Vector3d &){};

  // For joints
  inline virtual std::array<double, 7>
  Step(const franka::RobotState &, const Eigen::Matrix<double, 7, 1> &){};

  // For joints with computed-torque feedforward (desired velocity +
  // acceleration). Default ignores the feedforward channels and delegates to
  // the position-only joint Step, so every non-feedforward controller behaves
  // identically.
  inline virtual std::array<double, 7>
  Step(const franka::RobotState &robot_state,
       const Eigen::Matrix<double, 7, 1> &q_d,
       const Eigen::Matrix<double, 7, 1> &,
       const Eigen::Matrix<double, 7, 1> &) {
    return Step(robot_state, q_d);
  };

  inline void
  SetStateEstimator(const std::shared_ptr<estimator_utils::BaseStateEstimator>
                        &state_estimator_ptr) {
    this->state_estimator_ptr_ = state_estimator_ptr;
  };

  inline void ResetStateEstimator() { this->state_estimator_ptr_->Reset(); }

  inline virtual void EstimateVelocities(const franka::RobotState &,
                          std::shared_ptr<StateInfo> &) {};
};
} // NAMESPACE controller

#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_CONTROLLERS_BASE_CONTROLLER_H_

#pragma GCC diagnostic push
