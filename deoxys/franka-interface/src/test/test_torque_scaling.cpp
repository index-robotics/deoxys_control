// TorqueScaleToLimits: within limits, same direction, untouched when inside.
// Standalone (Eigen only, via control_utils.h), from deoxys/:
//   g++ -std=c++17 -I franka-interface/include -I /usr/include/eigen3 franka-interface/src/test/test_torque_scaling.cpp
#include <cmath>
#include <cstdio>

#include "utils/control_utils.h"

int main() {
  const std::array<double, 7> max = {30, 30, 30, 30, 8, 8, 8};
  const std::array<double, 7> min = {-30, -30, -30, -30, -8, -8, -8};
  int failures = 0;
  auto check = [&](std::array<double, 7> tau, double expect_scale) {
    std::array<double, 7> out = tau;
    control_utils::TorqueScaleToLimits(out, min, max);
    for (size_t i = 0; i < 7; i++) {
      if (out[i] > max[i] + 1e-12 || out[i] < min[i] - 1e-12 ||
          std::abs(out[i] - expect_scale * tau[i]) > 1e-9) {
        std::printf("joint %zu: %g -> %g (want scale %g)\n", i, tau[i], out[i],
                    expect_scale);
        ++failures;
      }
    }
  };
  check({5, -12, 3, 20, -1, 2, 0.5}, 1.0);        // inside: untouched
  check({5, -45, 3, 20, -1, 2, 0.5}, 30.0 / 45);  // j2 over: all scale by 2/3
  check({5, 40, 3, 20, -12, 2, 0.5}, 8.0 / 12);   // wrist binds harder than j2
  std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
  return failures ? 1 : 0;
}
