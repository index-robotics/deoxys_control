// Copyright 2022 Yifeng Zhu

#include "base_traj_interpolator.h"
#include <Eigen/Dense>
#ifndef DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSE_TRAJ_INTERPOLATOR_H_
#define DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSE_TRAJ_INTERPOLATOR_H_

namespace traj_utils {
class LinearPoseTrajInterpolator : public BaseTrajInterpolator {
private:
  Eigen::Vector3d p_start_;
  Eigen::Vector3d p_goal_;
  Eigen::Vector3d last_p_t_;
  Eigen::Vector3d prev_p_goal_;

  Eigen::Quaterniond q_start_;
  Eigen::Quaterniond q_goal_;
  Eigen::Quaterniond last_q_t_;
  Eigen::Quaterniond prev_q_goal_;

  // Feedforward [v; w; a; alpha], segment-chained and lerped like p_.
  // Zero-initialized so a pose-only Reset yields zero feedforward.
  using FF = Eigen::Matrix<double, 12, 1>;
  FF ff_start_ = FF::Zero();
  FF ff_goal_ = FF::Zero();
  FF last_ff_t_ = FF::Zero();

  double dt_;
  double last_time_;
  double max_time_;
  double start_time_;
  bool start_;
  bool first_goal_;

public:
  inline LinearPoseTrajInterpolator()
      : dt_(0.), last_time_(0.), max_time_(1.), start_time_(0.), start_(false),
        first_goal_(true){};

  inline ~LinearPoseTrajInterpolator(){};

  inline void Reset(const double &time_sec, const Eigen::Vector3d &p_start,
                    const Eigen::Quaterniond &q_start,
                    const Eigen::Vector3d &p_goal,
                    const Eigen::Quaterniond &q_goal, const int &policy_rate,
                    const int &rate,
                    const double &traj_interpolator_time_fraction) {

    dt_ = 1. / static_cast<double>(rate);
    last_time_ = time_sec;
    max_time_ =
        1. / static_cast<double>(policy_rate) * traj_interpolator_time_fraction;
    start_time_ = time_sec;

    start_ = false;

    if (first_goal_) {
      p_start_ = p_start;
      q_start_ = q_start;

      prev_p_goal_ = p_start;
      prev_q_goal_ = q_start;
      first_goal_ = false;
      // std::cout << "First goal" << std::endl;
    } else {
      // If the goal is already set, use prev goal as the starting point of
      // interpolation.
      prev_p_goal_ = p_goal_;
      prev_q_goal_ = q_goal_;

      p_start_ = prev_p_goal_;
      q_start_ = prev_q_goal_;
    }

    p_goal_ = p_goal;
    q_goal_ = q_goal;

    // Flip the sign if the dot product of quaternions is negative
    if (q_goal_.coeffs().dot(q_start_.coeffs()) < 0.0) {
      q_start_.coeffs() << -q_start_.coeffs();
    }
  };

  inline void GetNextStep(const double &time_sec, Eigen::Vector3d &p_t,
                          Eigen::Quaterniond &q_t) {

    if (!start_) {
      start_time_ = time_sec;
      last_p_t_ = p_start_;
      last_q_t_ = q_start_;
      start_ = true;
    }

    if (last_time_ + dt_ <= time_sec) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      last_p_t_ = p_start_ + t * (p_goal_ - p_start_);
      last_q_t_ = q_start_.slerp(t, q_goal_);
      last_time_ = time_sec;
    }
    p_t = last_p_t_;
    q_t = last_q_t_;
  };

  // Feedforward-aware Reset: the new segment's feedforward ramps from the
  // previous goal (zero on the first goal), then the pose setup is delegated.
  inline void Reset(const double &time_sec, const Eigen::Vector3d &p_start,
                    const Eigen::Quaterniond &q_start,
                    const Eigen::Vector3d &p_goal,
                    const Eigen::Quaterniond &q_goal,
                    const Eigen::Vector3d &v_goal,
                    const Eigen::Vector3d &w_goal,
                    const Eigen::Vector3d &a_goal,
                    const Eigen::Vector3d &alpha_goal, const int &policy_rate,
                    const int &rate,
                    const double &traj_interpolator_time_fraction) {
    if (first_goal_) {
      ff_start_.setZero();
    } else {
      ff_start_ = ff_goal_;
    }
    ff_goal_ << v_goal, w_goal, a_goal, alpha_goal;
    Reset(time_sec, p_start, q_start, p_goal, q_goal, policy_rate, rate,
          traj_interpolator_time_fraction);
  };

  // A new control session starts from rest: ramp the feedforward up from zero
  // instead of from a previous session's (or the same message's) goal.
  inline void ResetFeedforwardStart() {
    ff_start_.setZero();
    last_ff_t_.setZero();
  };

  // Feedforward-aware step: lerp (not ZOH) on the same throttled t, so per-ms
  // torque changes stay small instead of staircasing through libfranka's
  // post-filter rate limit (1 Nm/ms).
  inline void GetNextStep(const double &time_sec, Eigen::Vector3d &p_t,
                          Eigen::Quaterniond &q_t, Eigen::Vector3d &v_t,
                          Eigen::Vector3d &w_t, Eigen::Vector3d &a_t,
                          Eigen::Vector3d &alpha_t) {
    if (!start_) {
      last_ff_t_ = ff_start_;
    }
    const bool stepped = last_time_ + dt_ <= time_sec;
    GetNextStep(time_sec, p_t, q_t);
    if (stepped) {
      double t =
          std::min(std::max((time_sec - start_time_) / max_time_, 0.), 1.);
      last_ff_t_ = ff_start_ + t * (ff_goal_ - ff_start_);
    }
    v_t = last_ff_t_.segment<3>(0);
    w_t = last_ff_t_.segment<3>(3);
    a_t = last_ff_t_.segment<3>(6);
    alpha_t = last_ff_t_.segment<3>(9);
  };
};
} // namespace traj_utils
#endif // DEOXYS_FRANKA_INTERFACE_INCLUDE_UTILS_TRAJ_INTERPOLATORS_POSE_TRAJ_INTERPOLATOR_H_
