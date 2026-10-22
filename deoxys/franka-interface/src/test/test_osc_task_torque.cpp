// OscTaskTorque: with the full task inertia the end effector accelerates exactly
// as commanded, so a pure rotation leaves the EE position alone; the decoupled
// blocks push it. Standalone (Eigen only):
//   g++ -std=c++17 -I franka-interface/include -I /usr/include/eigen3 \
//     franka-interface/src/test/test_osc_task_torque.cpp -o /tmp/test_osc_task_torque
#include <cstdio>
#include <cstdlib>

#include "controllers/osc_task_torque.h"

int main() {
  std::srand(0);
  int failures = 0;
  for (int trial = 0; trial < 100; ++trial) {
    Eigen::Matrix<double, 7, 7> A = Eigen::Matrix<double, 7, 7>::Random();
    Eigen::Matrix<double, 7, 7> M =
        A * A.transpose() + 0.1 * Eigen::Matrix<double, 7, 7>::Identity();
    Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Random();
    Eigen::Matrix<double, 7, 7> M_inv = M.inverse();
    const Eigen::Vector3d a_p = Eigen::Vector3d::Random();
    const Eigen::Vector3d a_r = Eigen::Vector3d::Random();

    // EE acceleration from rest: J M^-1 tau.
    Eigen::Matrix<double, 6, 1> want;
    want << a_p, a_r;
    Eigen::Matrix<double, 6, 1> full =
        J * M_inv * controller::OscTaskTorque(J, M_inv, a_p, a_r, true);
    Eigen::Matrix<double, 6, 1> rot_only =
        J * M_inv *
        controller::OscTaskTorque(J, M_inv, Eigen::Vector3d::Zero(), a_r, true);
    Eigen::Matrix<double, 6, 1> rot_only_decoupled =
        J * M_inv *
        controller::OscTaskTorque(J, M_inv, Eigen::Vector3d::Zero(), a_r, false);

    if ((full - want).norm() > 1e-6 * want.norm() ||
        rot_only.head<3>().norm() > 1e-6 * a_r.norm()) {
      std::printf("trial %d: full law misses the commanded acceleration\n", trial);
      ++failures;
    }
    // Negative control: the decoupled law must show the coupling this fixes.
    if (rot_only_decoupled.head<3>().norm() < 1e-3 * a_r.norm()) {
      std::printf("trial %d: decoupled law shows no coupling\n", trial);
      ++failures;
    }
  }
  std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
  return failures ? 1 : 0;
}
