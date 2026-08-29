#!/usr/bin/env python3
"""
End-effector position accuracy test (Phase 9).

Two DISTINCT accuracy numbers are reported, on purpose:

1. Mathematical FK/IK accuracy - pure Python, no ROS/Gazebo involved.
   Measures whether inverse_kinematics(x,y,z) -> joint angles -> FK back to
   (x,y,z) is numerically self-consistent. This tests the math only.

2. Gazebo tracking accuracy - commands the REAL simulated robot (via
   joint_trajectory_controller) to each IK solution, waits for it to
   settle, reads back the ACTUAL joint_states Gazebo reports, and computes
   FK from those measured angles vs. the original Cartesian target. This
   captures controller tracking/settling error (gravity sag, gain
   settling), which the pure-math test above cannot see.

error = sqrt(dx^2 + dy^2 + dz^2), reported in millimeters, requirement: < 1 mm.
"""

import math
import time

import rclpy

import kinematics as kin
from pick_and_place import PickAndPlace

# All verified reachable in advance with kinematics.inverse_kinematics() -
# the arm's link lengths (0.30, 0.30, 0.10 m) create an unreachable inner
# annulus near the shoulder axis, so targets are not picked arbitrarily.
TEST_TARGETS = [
    (0.55, 0.00, 0.015),
    (0.55, 0.00, 0.150),
    (0.4213, 0.3535, 0.015),
    (0.4213, 0.3535, 0.250),
    (0.60, 0.00, 0.250),
    (0.35, 0.00, 0.450),
]

SETTLE_EXTRA_S = 1.5  # extra time after the action reports "done" for gravity/gain settling


def print_math_accuracy():
    print("=" * 70)
    print("1) MATHEMATICAL FK/IK ACCURACY (pure Python, no Gazebo)")
    print("=" * 70)
    fk_err = kin.self_test()
    print(f"  FK cross-check (D-H vs. closed-form): max error = {fk_err * 1000:.9f} mm")

    max_err_m, failures = kin.ik_roundtrip_test()
    print(f"  IK round-trip (FK -> IK -> FK):        max error = {max_err_m * 1000:.9f} mm "
          f"({failures} unreachable samples skipped)")
    print()


def run_gazebo_accuracy_test():
    print("=" * 70)
    print("2) GAZEBO TRACKING ACCURACY (live simulated robot)")
    print("=" * 70)

    rclpy.init()
    node = PickAndPlace()

    results = []
    try:
        for target in TEST_TARGETS:
            sol = kin.inverse_kinematics(*target)
            if sol is None:
                print(f"  target {target}: UNREACHABLE, skipped")
                continue

            node.move_arm_to(sol, duration_s=2.5, label=f"accuracy target {target}")
            time.sleep(SETTLE_EXTRA_S)
            rclpy.spin_once(node, timeout_sec=0.2)

            achieved = kin.forward_kinematics(*node.current_q)
            err_mm = math.dist(target, achieved) * 1000.0
            results.append((target, sol, achieved, err_mm))

            status = "PASS" if err_mm < 1.0 else "FAIL"
            print(f"  target={tuple(round(v,4) for v in target)}  "
                  f"commanded_q={tuple(round(v,4) for v in sol)}  "
                  f"achieved_xyz={tuple(round(v,5) for v in achieved)}  "
                  f"error={err_mm:.4f} mm  [{status}]")

        node.move_arm_to((0.0, 0.0, 0.0), duration_s=2.0, label="return home")
    finally:
        rclpy.shutdown()

    print()
    if results:
        errs = [r[3] for r in results]
        print(f"  Samples: {len(errs)}   Max error: {max(errs):.4f} mm   "
              f"Mean error: {sum(errs)/len(errs):.4f} mm")
        overall = "PASS (<1 mm)" if max(errs) < 1.0 else "FAIL (>=1 mm)"
        print(f"  Overall requirement (<1 mm): {overall}")
    else:
        print("  No reachable targets were tested.")
    print()
    return results


if __name__ == "__main__":
    print_math_accuracy()
    run_gazebo_accuracy_test()
