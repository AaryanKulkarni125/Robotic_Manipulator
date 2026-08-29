# manipulator_ws

A 3-DOF (R-R-R) robotic arm, simulated end-to-end in Gazebo Sim with ROS 2: URDF/xacro model,
ros2_control-based joint control, hand-derived analytic kinematics, cubic trajectory generation,
a 2-finger gripper, and an autonomous pick-and-place demo.

## Tech Stack

| Layer | Tech |
|---|---|
| Middleware | ROS 2 Jazzy |
| Simulation | Gazebo Sim (gz-sim 8, "Harmonic"-class) — not Gazebo Classic |
| Robot description | URDF via Xacro (single source of truth; `.urdf` is a build artifact only) |
| Control | `ros2_control` + `ros2_controllers` + `gz_ros2_control` (`GazeboSimSystem` hardware plugin) |
| Controllers | `joint_state_broadcaster`, 2x `joint_trajectory_controller` (arm, gripper) |
| Kinematics | Pure NumPy — analytic D-H forward kinematics and closed-form inverse kinematics, no MoveIt |
| Grasping | Gazebo's `DetachableJoint` system (real physics attach/detach, not scripted) |
| Demo logic | `rclpy`, `FollowJointTrajectory` action client |
| Build | `ament_cmake`, `colcon` |

## Package Layout

```
src/manipulator_description/
├── urdf/                   # xacro model: links, joints, materials, inertials, gazebo/ros2_control plugins, gripper
├── config/
│   ├── manipulator_controllers.yaml   # controller_manager + joint_trajectory_controller config
│   └── display.rviz
├── launch/
│   ├── display.launch.py   # RViz only
│   ├── gazebo.launch.py    # Gazebo + arm control, no gripper/object
│   └── demo.launch.py      # full system: arm + gripper + object (supports headless:=true)
├── worlds/
│   └── pick_object.sdf     # the object the demo picks up
├── scripts/
│   ├── kinematics.py            # D-H + geometric forward kinematics, analytic inverse kinematics
│   ├── trajectory_generator.py  # cubic-polynomial joint trajectories (zero-velocity boundaries)
│   ├── pick_and_place.py        # autonomous pick-and-place state machine
│   ├── gripper_bridge.py        # gripper attach/detach topic bridge
│   └── accuracy_test.py         # measures FK/IK math accuracy and live Gazebo tracking accuracy
└── docs/
    ├── PROJECT_DOCUMENTATION.md # architecture, DH parameters, kinematics derivation, validation methodology
    └── CHANGELOG.md             # phase-by-phase build log, bugs found and fixed, root causes
```

## Robot

3-DOF articulated arm: waist (Z, ±180°) → shoulder (Y, ±90°) → elbow (Y, ±135°), plus a 2-finger
prismatic gripper (0.02 m stroke) at the end effector. Full dimensions/masses and D-H parameters
are in `docs/PROJECT_DOCUMENTATION.md`.

## Kinematics

Forward kinematics is implemented twice — a literal D-H matrix chain and an independently-derived
closed-form geometric solution — and cross-checked against each other. Inverse kinematics is
closed-form analytic (waist decoupled from a 2-link planar sub-problem), tries both elbow-up and
elbow-down branches, and returns `None` for unreachable targets. Verified to machine precision
(round-trip FK→IK→FK over 500 random samples).

## Grasping / Pick-and-Place

`pick_and_place.py` runs: `HOME → APPROACH → GRASP → CLOSE_GRIPPER → ATTACH → LIFT → TRANSPORT →
PLACE → DETACH → OPEN_GRIPPER → RETURN_HOME → DONE`, using Gazebo's real `DetachableJoint` plugin
for the grasp (not a faked pickup). See `docs/CHANGELOG.md` for the full debugging history of
getting this to track reliably in this gz-sim build.

## Accuracy

Measured, not assumed: mathematical FK/IK error is 0.0 mm at machine precision, and live Gazebo
tracking error (IK-commanded angles vs. FK computed from the *actually reported* `/joint_states`)
is 0.0000 mm across 6 test targets — both well under the 1 mm requirement. See
`scripts/accuracy_test.py` and `docs/PROJECT_DOCUMENTATION.md` §8.

## Running

```bash
cd ~/manipulator_ws
colcon build --symlink-install
source install/setup.bash

# RViz only
ros2 launch manipulator_description display.launch.py

# Gazebo + arm control, no gripper/object
ros2 launch manipulator_description gazebo.launch.py

# Full system: arm + gripper + object
ros2 launch manipulator_description demo.launch.py
ros2 launch manipulator_description demo.launch.py headless:=true   # server-only, no GUI

# Autonomous pick-and-place demo (after demo.launch.py is up and settled)
ros2 run manipulator_description pick_and_place.py

# Accuracy test
ros2 run manipulator_description accuracy_test.py
```

`headless:=true` is recommended for reliable testing — this environment's Gazebo GUI is CPU-bound
(no GPU passthrough) and has a known, harmless, GUI-only object-rendering quirk documented in
`docs/CHANGELOG.md`.

## Further Reading

- `docs/PROJECT_DOCUMENTATION.md` — architecture, D-H parameters, kinematics derivation, gripper
  ordering requirements, accuracy validation methodology.
- `docs/CHANGELOG.md` — phase-by-phase build log: every bug found, its root cause, and the fix.
