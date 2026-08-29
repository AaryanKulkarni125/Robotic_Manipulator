# 3-DOF R-R-R Manipulator — Project Documentation

## 1. System Architecture

```
world --fixed--> base_link --joint1(Z)--> link1 --joint2(Y)--> link2 --joint3(Y)--> end_effector --fixed--> ee_tip
                                                                                                        |
                                                                                     left_finger_joint --+-- right_finger_joint
```

- **Description**: `urdf/manipulator.xacro` includes `materials.xacro`, `inertial_macros.xacro`, `gazebo.xacro`,
  `ros2_control.xacro`, and `gripper.xacro`. Xacro is the single source of truth; `manipulator.urdf` is never
  hand-edited (it is only a `xacro >` build artifact for validation).
- **Simulation**: Gazebo Sim (gz-sim 8, "Harmonic"-class), not Gazebo Classic.
- **Control**: `gz_ros2_control` (`GazeboSimSystem` hardware plugin) + `ros2_control` + `ros2_controllers`
  (`joint_state_broadcaster`, two `joint_trajectory_controller` instances: arm + gripper).
- **Kinematics/planning**: Python scripts (`scripts/`) using pure NumPy — no external kinematics library.
- **Demonstration**: `scripts/pick_and_place.py`, an rclpy node driving both controllers via
  `FollowJointTrajectory` actions.

## 2. Robot Specification

| Property | Value |
|---|---|
| Configuration | 3-DOF articulated R-R-R |
| Joint 1 (waist) | Revolute, axis Z, ±180°, effort 10, velocity 1.0 |
| Joint 2 (shoulder) | Revolute, axis Y, ±90°, effort 10, velocity 1.0 |
| Joint 3 (elbow) | Revolute, axis Y, ±135°, effort 5, velocity 1.0 |
| Base | height 0.15 m, radius 0.10 m, mass 1.5 kg |
| Link 1 | length 0.30 m, radius 0.040 m, mass 0.8 kg |
| Link 2 | length 0.30 m, radius 0.035 m, mass 0.6 kg |
| End effector | length 0.10 m, radius 0.025 m, mass 0.2 kg |
| Gripper | 2 prismatic fingers, 0.02 m stroke each |

## 3. D-H Parameters (scripts/kinematics.py)

Standard DH transform: `T_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)`

| i | a_i (m) | alpha_i | d_i (m) | theta_i |
|---|---|---|---|---|
| 1 | 0.30 (link1) | +90° | 0.15 (base_height) | q1 |
| 2 | 0.30 (link2) | 0° | 0 | q2 |
| 3 | 0.10 (ee_length) | 0° | 0 | q3 |

Key modeling decision: each link's length `a_i` is attached to the transform whose `theta_i` is the joint
**before** that link (e.g. link1's length rotates with joint1/waist, not joint2) — this was derived directly
from the xacro's joint-origin/axis chain, not copied from a generic table, and cross-validated (see §8).

## 4. Forward Kinematics

Two independent implementations, cross-checked against each other and against live Gazebo poses:

- `dh_fk(q1,q2,q3)` — literal D-H matrix product from the table above.
- `geometric_fk(q1,q2,q3)` — closed form derived directly from the xacro chain:
  ```
  r     = L1 + L2*cos(q2) + L3*cos(q2+q3)
  z_rel = L2*sin(q2) + L3*sin(q2+q3)
  x = r*cos(q1), y = r*sin(q1), z = base_height + z_rel
  ```
  (L1=link1_length, L2=link2_length, L3=ee_length)

`forward_kinematics()` uses `geometric_fk` and is the target reference point for IK — the physical tip
`ee_tip`, `ee_length` beyond joint3.

## 5. Inverse Kinematics

Analytic, closed-form (waist decoupled from a 2-link planar sub-problem):

```
q1 = atan2(y, x)
r' = hypot(x,y) - L1 ;  z' = z - base_height
D  = (r'^2 + z'^2 - L2^2 - L3^2) / (2*L2*L3)
q3 = atan2(±sqrt(1-D^2), D)              # elbow up/down
q2 = atan2(z', r') - atan2(L3*sin(q3), L2 + L3*cos(q3))
```

- Unreachable targets (geometric or joint-limit) return `None`.
- Both elbow-up/elbow-down branches are tried automatically before giving up.
- **Reachability note**: because L2 (0.30 m) and L3 (0.10 m) differ substantially, there is an
  unreachable *inner annulus* of radius < |L2-L3| = 0.20 m and an outer limit of L2+L3 = 0.40 m,
  both measured from the point offset L1=0.30 m out from the waist axis. Targets must be checked
  with `inverse_kinematics()` before use — several first-draft demo positions were rejected and
  corrected during Phase 8 for exactly this reason.

## 6. Trajectory Generation (Phase 6)

`scripts/trajectory_generator.py` — cubic polynomial, zero-velocity boundary conditions:
```
q(t)    = q0 + 3*(qf-q0)*(t/T)^2 - 2*(qf-q0)*(t/T)^3
qdot(t) = 6*(qf-q0)*t*(T-t)/T^3
```
Produces a dense multi-point `JointTrajectory` (20 waypoints by default), sent to
`joint_trajectory_controller`'s `FollowJointTrajectory` action. Verified standalone: position exactly
matches the goal at t=T, velocity ≈0 at both endpoints.

## 7. Gripper

Two independent prismatic fingers (`left_finger_joint`, `right_finger_joint`) mounted on `ee_tip`.
**Important, empirically-discovered ordering requirement**: `right_finger_joint` must be listed
*before* `left_finger_joint` in both `ros2_control.xacro` and `config/manipulator_controllers.yaml`'s
`gripper_controller.joints` list — with the reverse order, `left_finger_joint` silently never tracked
position commands in this gz-sim/gz_ros2_control build, confirmed by reading the finger's actual
physical pose (not just the ros2_control-reported state) via `gz topic -e -t .../dynamic_pose/info`.
A `<mimic>` joint was tried first and rejected: this DART physics build does not support mimic
constraints (`[Err] Physics.cc:1808 ... does not support mimic constraints`).

## 8. Accuracy Validation Methodology (Phase 9)

Two distinct numbers are measured — deliberately not conflated:

1. **Mathematical FK/IK accuracy** (pure Python): `dh_fk` vs `geometric_fk` cross-check over 500 random
   configurations, and `FK -> IK -> FK` round-trip over another 500. No Gazebo involved.
2. **Gazebo tracking accuracy** (live robot): for each Cartesian target, IK produces joint angles, the
   angles are sent via `joint_trajectory_controller`, the script waits for the trajectory + 1.5 s extra
   settling time, then reads the **actual** `/joint_states` Gazebo reports and computes FK from those
   measured angles — not the commanded ones.

`error = sqrt(dx^2 + dy^2 + dz^2)`, reported in mm.

**Result (measured, `scripts/accuracy_test.py`, 6 diverse reachable targets)**:
- Mathematical FK/IK: 0.000000000 mm (machine precision)
- Gazebo tracking: 0.0000 mm on all 6 targets — well under the 1 mm requirement.

This precision was not automatic: the default `gz_ros2_control` position-interface gain
(`position_proportional_gain = 0.1`) was far too low for this arm's gravity-loaded joints to settle
to sub-mm accuracy. It was raised to `500.0` in `config/manipulator_controllers.yaml` after the first
test run showed a measurable steady-state gravity-induced error, then re-verified.

## 9. Pick-and-Place Architecture

`scripts/pick_and_place.py` runs an explicit state sequence:

```
HOME -> APPROACH -> GRASP_POSITION -> CLOSE_GRIPPER -> ATTACH -> LIFT
  -> TRANSPORT -> PLACE -> DETACH -> OPEN_GRIPPER -> RETURN_HOME -> DONE
```

Arm motion between named waypoints is exactly the original, previously-verified trajectory: same
Cartesian targets (`OBJECT_POS`/`DEST_POS` exactly, `APPROACH_CLEARANCE` above them), same default
IK branch, same durations, same order. The only functional change relative to the original script
is what happens at `ATTACH`/`DETACH` (see below) — no waypoint, IK branch, or timing was altered to
accommodate it. A session that redesigned the trajectory around suspected (but not user-verified)
collisions was reverted in full; see §11's last entries for what was tried and undone.

**Grasp/carry mechanism**: gz-sim's `DetachableJoint` system (`gz-sim-detachable-joint-system`,
configured in `urdf/gripper.xacro`) is the real, temporary Gazebo attachment — publishing an Empty
message on `/gripper/attach` creates an actual fixed joint between `end_effector` and the object's
`object_link`; a message on `/gripper/detach` removes it. `/gripper/state` (a `StringMsg` the plugin
publishes back) is the authoritative source of truth for whether the object is currently grasped,
and `pick_and_place.py` waits for and confirms it at every attach/detach rather than assuming success.

Two limitations in this specific installed build (gz-sim 8.11.0 via `ros-jazzy-gz-sim-vendor`) were
found and worked around, both documented in detail in `pick_and_place.py`'s module docstring:

1. The plugin does not reliably start detached in practice, so the very first thing the node does
   (before the state machine begins) is send one detach request to force a known-good baseline.
2. The resulting joint does not propagate the parent's motion to the attached object for this arm's
   kinematically-driven joint chain (verified directly against gz-sim's own official
   `detachable_joint.sdf` example, where the identical mechanism *does* work for a dynamics-driven
   vehicle) — and a `set_pose` correction was found to be silently ignored while the object is
   attached (DART will not kinematically reposition a body that is currently rigidly joint-constrained).
   The accepted workaround (an explicit, user-approved trade-off) is `_sync_held_object_pose()`:
   once per completed arm move while holding, it briefly detaches, snaps the object's pose to the
   gripper's own FK tool position via the `/world/empty/set_pose` service, and re-attaches. This is a
   handful of calls total across a full demo, each tied to an already-planned motion completing, not
   a fixed-rate polling/teleport loop — but it does mean the object visibly steps between waypoints
   rather than continuously tracking the gripper mid-move. If a future Gazebo/gz_ros2_control version
   fixes DetachableJoint's motion propagation for actuated chains, `_sync_held_object_pose()` and its
   call sites can simply be deleted with no other changes.

**Known object**: the object's position is read from `worlds/pick_object.sdf`'s spawn pose
(`OBJECT_POS` in `pick_and_place.py`) — no perception/vision is used, per the project's scope.

## 10. Running the System

```bash
cd ~/manipulator_ws
colcon build --symlink-install
source install/setup.bash

# RViz only (Phase 1)
ros2 launch manipulator_description display.launch.py

# Gazebo + arm control, no gripper/object (Phase 3 checkpoint)
ros2 launch manipulator_description gazebo.launch.py

# Full system: arm + gripper + object, headless option for testing
ros2 launch manipulator_description demo.launch.py
ros2 launch manipulator_description demo.launch.py headless:=true   # server-only, no GUI

# Run the autonomous pick-and-place demo (after demo.launch.py is up and settled)
ros2 run manipulator_description pick_and_place.py

# Run the accuracy test
ros2 run manipulator_description accuracy_test.py
```

## 11. Troubleshooting Notes From This Build

- **`launch_ros` "Unable to parse robot_description as yaml"**: happens once the xacro output gets large
  enough. Fix: wrap the `Command(...)` substitution in
  `launch_ros.parameter_descriptions.ParameterValue(..., value_type=str)`.
- **`gz_spawn_model.launch.py`'s `-file` mode ignores the model SDF's own `<pose>` tag** — it defaults
  `x`/`y`/`z` to `0.0` and passes them explicitly, overriding the file. Always pass `x`/`y`/`z`
  launch arguments explicitly when spawning from a file.
- **sdformat lumps any link connected only via a `type="fixed"` joint into its parent** during
  URDF→SDF conversion, *even if the link has a `<collision>`* — this removes it as a distinct SDF
  `<link>` (it survives only as a `<frame>`). A Gazebo plugin that needs to reference such a link by
  name (e.g. `DetachableJoint`'s `parent_link`) must reference the *parent* it was lumped into instead.
  Verify with `gz sdf -p <urdf-file>` and grep for the link name.
- **FastDDS + WSL2 + many spawn/kill cycles in one long session**: stale `/dev/shm/fastrtps_*` segments
  and a stale `ros2 daemon` graph cache can cause `ros2 control` / service calls to fail with
  `rcl node's context is invalid` even though the actual controller_manager is healthy. Fix:
  `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` and `ros2 daemon stop && ros2 daemon start`
  between runs during heavy iterative testing.
- **A redesign pass changed the trajectory (IK branch, added transit waypoints, changed grasp/place
  descent heights) to work around collisions that geometric analysis predicted but that did not
  reliably reproduce as actual object-launching contact when tested live** (a clean run with the
  original trajectory showed the object sitting motionless through HOME/APPROACH/GRASP_POSITION/
  CLOSE_GRIPPER, with no collision). Per explicit instruction, this redesign was fully reverted:
  `pick_and_place.py` was restored to the original waypoints/IK branch/gripper-closed value, and
  `manipulator.xacro`/`gripper.xacro`'s collision-geometry changes (shortened `end_effector`
  collision, removed `ee_tip` collision) were reverted too. The only lesson worth keeping from that
  detour: if a real, reproducible collision is ever found with the *current* (original) trajectory,
  re-derive the fix from live evidence (a continuous Gazebo pose trace during the actual failure),
  not from geometric prediction alone - and treat any such fix as a deliberate, flagged decision,
  not something to bundle into an unrelated change.
