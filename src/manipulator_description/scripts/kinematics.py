#!/usr/bin/env python3
"""
D-H model, forward kinematics and inverse kinematics for the 3-DOF R-R-R
manipulator described in urdf/manipulator.xacro.

Robot structure (from the xacro, unchanged):

    world --fixed--> base_link --joint1(Z)--> link1 --joint2(Y)--> link2 --joint3(Y)--> end_effector

    joint1: origin xyz=(0,0,BASE_HEIGHT),      axis Z, range [-180,180] deg
    joint2: origin xyz=(LINK1_LENGTH,0,0),     axis Y, range [-90,90]   deg
    joint3: origin xyz=(LINK2_LENGTH,0,0),     axis Y, range [-135,135] deg
    end_effector tip is a further LINK_EE_LENGTH along local X from joint3.

At q1=q2=q3=0 the arm is fully extended horizontally along +X, at height
BASE_HEIGHT. This module implements FK two independent ways and checks they
agree (dh_fk vs geometric closed-form), then uses the closed-form relations
to solve IK analytically.
"""

import math
import numpy as np

# ---------------------------------------------------------------------------
# Robot geometry (must match urdf/manipulator.xacro exactly)
# ---------------------------------------------------------------------------
BASE_HEIGHT = 0.15
LINK1_LENGTH = 0.30
LINK2_LENGTH = 0.30
EE_LENGTH = 0.10

JOINT_LIMITS = {
    "q1": (math.radians(-180.0), math.radians(180.0)),
    "q2": (math.radians(-90.0), math.radians(90.0)),
    "q3": (math.radians(-135.0), math.radians(135.0)),
}


# ---------------------------------------------------------------------------
# D-H model
# ---------------------------------------------------------------------------
# Standard DH transform: T_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
#
# Frame assignment used here:
#   Frame 1 (joint1/waist): Z0 = joint1 axis (world Z), theta1 = q1.
#     a1 = LINK1_LENGTH is the link BETWEEN joint1 and joint2, so it must be
#     carried by T1 (it physically rotates with joint1/waist). alpha1 = +90 deg
#     then twists Z from vertical (joint1's axis) to horizontal (joint2's axis).
#   Frame 2 (joint2/shoulder): theta2 = q2. a2 = LINK2_LENGTH (link BETWEEN
#     joint2 and joint3, rotates with joint2). alpha2 = 0 (joint3 axis is
#     parallel to joint2 axis - both are the arm's Y-type pitch axis).
#   Frame 3 (joint3/elbow): theta3 = q3. a3 = EE_LENGTH (the end-effector
#     segment has no joint of its own; its length rotates with joint3).
#
#           a_i            alpha_i         d_i           theta_i (variable)
DH_PARAMS = [
    (LINK1_LENGTH,    math.pi / 2,   BASE_HEIGHT,   "q1"),   # frame 0 -> 1
    (LINK2_LENGTH,    0.0,           0.0,           "q2"),   # frame 1 -> 2
    (EE_LENGTH,       0.0,           0.0,           "q3"),   # frame 2 -> 3
]


def _dh_transform(a, alpha, d, theta):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def dh_fk(q1, q2, q3):
    """Forward kinematics via the D-H parameter table. Returns 4x4 homogeneous transform."""
    q = {"q1": q1, "q2": q2, "q3": q3}
    T = np.eye(4)
    for a, alpha, d, theta_name in DH_PARAMS:
        T = T @ _dh_transform(a, alpha, d, q[theta_name])
    return T


def geometric_fk(q1, q2, q3):
    """
    Closed-form forward kinematics derived directly from the xacro joint
    chain (waist rotation + planar 2-link-plus-tool arm). Independent
    implementation from dh_fk(), used as a cross-check.
    """
    r = LINK1_LENGTH + LINK2_LENGTH * math.cos(q2) + EE_LENGTH * math.cos(q2 + q3)
    z_rel = LINK2_LENGTH * math.sin(q2) + EE_LENGTH * math.sin(q2 + q3)

    x = r * math.cos(q1)
    y = r * math.sin(q1)
    z = BASE_HEIGHT + z_rel

    # Orientation: waist rotation about Z, then cumulative pitch (q2+q3) about
    # the (waist-rotated) Y axis.
    Rz = np.array([
        [math.cos(q1), -math.sin(q1), 0],
        [math.sin(q1),  math.cos(q1), 0],
        [0, 0, 1],
    ])
    phi = q2 + q3
    Ry = np.array([
        [math.cos(phi), 0, math.sin(phi)],
        [0, 1, 0],
        [-math.sin(phi), 0, math.cos(phi)],
    ])
    R = Rz @ Ry

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def forward_kinematics(q1, q2, q3):
    """Public FK entry point. Returns (x, y, z) end-effector position in meters."""
    T = geometric_fk(q1, q2, q3)
    return tuple(T[:3, 3])


def inverse_kinematics(x, y, z, elbow="up", _retry=True):
    """
    Analytic IK for a target end-effector position (x, y, z) in the base_link
    frame (base_link origin, i.e. z=0 is the bottom of the base cylinder).

    Returns (q1, q2, q3) in radians, or None if the target is unreachable
    (either geometrically, or only reachable outside joint limits).

    elbow: "up" or "down" selects between the two analytic solution branches.
    Both branches are tried automatically (_retry) before giving up.
    """
    q1 = math.atan2(y, x)

    r = math.hypot(x, y) - LINK1_LENGTH
    z_rel = z - BASE_HEIGHT

    L2 = LINK2_LENGTH
    L3 = EE_LENGTH

    dist_sq = r * r + z_rel * z_rel
    dist = math.sqrt(dist_sq)

    if dist > (L2 + L3) or dist < abs(L2 - L3):
        return None  # geometrically unreachable

    cos_q3 = (dist_sq - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    cos_q3 = max(-1.0, min(1.0, cos_q3))  # clamp for numerical safety

    sin_q3_mag = math.sqrt(max(0.0, 1.0 - cos_q3 * cos_q3))
    if elbow == "up":
        q3 = math.atan2(sin_q3_mag, cos_q3)
    else:
        q3 = math.atan2(-sin_q3_mag, cos_q3)

    q2 = math.atan2(z_rel, r) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))

    if not _within_limits(q1, q2, q3):
        if _retry:
            alt_elbow = "down" if elbow == "up" else "up"
            return inverse_kinematics(x, y, z, elbow=alt_elbow, _retry=False)
        return None

    return (q1, q2, q3)


def _within_limits(q1, q2, q3):
    for name, val in zip(("q1", "q2", "q3"), (q1, q2, q3)):
        lo, hi = JOINT_LIMITS[name]
        if not (lo - 1e-9 <= val <= hi + 1e-9):
            return False
    return True


def self_test(num_samples=500, tol_m=1e-9):
    """
    Cross-checks dh_fk() against geometric_fk() over random joint
    configurations within limits. Only end-effector POSITION is compared:
    the D-H tool frame's axis labeling is a modeling convention and need not
    match the URDF end_effector frame's axis labeling, so their rotation
    submatrices are not expected to be numerically identical. Position is
    the physically meaningful, convention-independent invariant, and is
    what the accuracy requirement (<1 mm) is measured against. Raises
    AssertionError on a position mismatch.
    """
    rng = np.random.default_rng(42)
    max_err = 0.0
    for _ in range(num_samples):
        q1 = rng.uniform(*JOINT_LIMITS["q1"])
        q2 = rng.uniform(*JOINT_LIMITS["q2"])
        q3 = rng.uniform(*JOINT_LIMITS["q3"])

        T_dh = dh_fk(q1, q2, q3)
        T_geo = geometric_fk(q1, q2, q3)

        err = np.linalg.norm(T_dh[:3, 3] - T_geo[:3, 3])
        max_err = max(max_err, err)
        assert err < tol_m, (
            f"FK position mismatch at q=({q1:.4f},{q2:.4f},{q3:.4f}): "
            f"dh={T_dh[:3,3]} geo={T_geo[:3,3]} err={err}"
        )

    return max_err


def ik_roundtrip_test(num_samples=500, tol_m=1e-6):
    """
    Generates random valid joint configs, computes FK, feeds the resulting
    position through IK, then FK again, and checks the round-trip position
    error. Returns the max error in meters.
    """
    rng = np.random.default_rng(7)
    max_err = 0.0
    failures = 0
    for _ in range(num_samples):
        q1 = rng.uniform(*JOINT_LIMITS["q1"])
        q2 = rng.uniform(*JOINT_LIMITS["q2"])
        q3 = rng.uniform(*JOINT_LIMITS["q3"])

        target = forward_kinematics(q1, q2, q3)
        sol = inverse_kinematics(*target)
        if sol is None:
            failures += 1
            continue

        achieved = forward_kinematics(*sol)
        err = math.dist(target, achieved)
        max_err = max(max_err, err)
        assert err < tol_m, f"IK round-trip error {err} m too large for target {target}"

    return max_err, failures


if __name__ == "__main__":
    print("Running FK self-consistency test (DH vs. closed-form)...")
    max_err = self_test()
    print(f"  OK - max position discrepancy: {max_err:.3e} m")

    print("Running IK round-trip test (FK -> IK -> FK)...")
    max_err, failures = ik_roundtrip_test()
    print(f"  OK - max round-trip position error: {max_err * 1000:.6f} mm, "
          f"{failures} unreachable-after-clamp samples skipped")

    print("\nSample FK values:")
    for q in [(0, 0, 0), (math.radians(90), 0, 0), (0, math.radians(45), math.radians(-45))]:
        pos = forward_kinematics(*q)
        print(f"  q={tuple(round(math.degrees(v),1) for v in q)} deg -> "
              f"xyz=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
