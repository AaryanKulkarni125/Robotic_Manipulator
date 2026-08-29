#!/usr/bin/env python3
"""
Cubic polynomial joint-space trajectory generation (Phase 6).

Given a start and goal joint configuration and a duration, produces a
trajectory_msgs/JointTrajectory with intermediate waypoints computed from
the standard zero-velocity-boundary cubic polynomial:

    q(t)    = q0 + 3*(qf - q0)*(t/T)^2 - 2*(qf - q0)*(t/T)^3
    qdot(t) = 6*(qf - q0)*t*(T - t) / T^3

This guarantees:
  - position continuity (q(0)=q0, q(T)=qf exactly)
  - smooth velocity (qdot(0)=qdot(T)=0, continuous in between)
  - each joint reaches its goal at the same time T, moving independently
    along its own cubic (standard joint-space trajectory generation)

The resulting multi-point JointTrajectory is sent to a
joint_trajectory_controller, which itself performs spline interpolation
between the supplied waypoints - the waypoints here are dense enough that
the controller's inter-point interpolation error is negligible.
"""

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def cubic_position(q0, qf, t, T):
    if T <= 0.0:
        return qf
    tau = t / T
    return q0 + (qf - q0) * (3 * tau ** 2 - 2 * tau ** 3)


def cubic_velocity(q0, qf, t, T):
    if T <= 0.0:
        return 0.0
    return (qf - q0) * (6 * t * (T - t)) / (T ** 3)


def build_cubic_trajectory(joint_names, q0, qf, duration_s, num_waypoints=20):
    """
    joint_names: list[str]
    q0, qf: list[float], same length as joint_names
    duration_s: float, total trajectory time
    num_waypoints: number of intermediate points (>=2, includes the goal)

    Returns a trajectory_msgs/msg/JointTrajectory.
    """
    assert len(q0) == len(qf) == len(joint_names)
    num_waypoints = max(2, num_waypoints)

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)

    for i in range(1, num_waypoints + 1):
        t = duration_s * i / num_waypoints
        point = JointTrajectoryPoint()
        point.positions = [cubic_position(a, b, t, duration_s) for a, b in zip(q0, qf)]
        point.velocities = [cubic_velocity(a, b, t, duration_s) for a, b in zip(q0, qf)]
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points.append(point)

    return traj


if __name__ == "__main__":
    # Standalone sanity check (no ROS required): verify boundary conditions.
    q0 = [0.0, 0.0, 0.0]
    qf = [0.5, -0.3, 0.2]
    T = 2.0
    traj = build_cubic_trajectory(["j1", "j2", "j3"], q0, qf, T, num_waypoints=10)

    first, last = traj.points[0], traj.points[-1]
    print(f"waypoints: {len(traj.points)}")
    print(f"last point positions: {last.positions} (expected {qf})")
    print(f"first point velocities: {first.velocities}")
    print(f"last point velocities: {last.velocities} (expected ~0 at t=T)")

    for a, b in zip(last.positions, qf):
        assert abs(a - b) < 1e-9
    for v in last.velocities:
        assert abs(v) < 1e-6
    print("OK - cubic trajectory boundary conditions verified")
