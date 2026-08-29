# CHANGELOG

## Phase 3 — ros2_control + gz_ros2_control + controllers
- Added `urdf/ros2_control.xacro` (hardware interfaces for joint1-3, `gz_ros2_control::GazeboSimROS2ControlPlugin`).
- Added `config/manipulator_controllers.yaml` (`joint_state_broadcaster`, `joint_trajectory_controller`).
- Added `launch/gazebo.launch.py`. Verified: joint1 commanded to 0.5 rad, matched by both
  `/joint_states` and Gazebo's own link pose.

## Phase 4-5 — D-H model, forward/inverse kinematics
- Added `scripts/kinematics.py`. Initial D-H table had `a_i`/`theta_i` mismatched by one joint
  (a link's length was attached to the wrong joint's transform) — caught by the built-in
  `dh_fk` vs `geometric_fk` cross-check (`self_test()`), which threw a large position mismatch on
  the first run. Fixed by re-deriving which transform each link length belongs to. After fix:
  max discrepancy 2.29e-16 m (machine precision).
- IK derived analytically from the corrected geometric model (waist decoupled 2-link planar
  sub-problem). Round-trip FK→IK→FK verified to 0.000000 mm over 500 random samples.

## Phase 6 — Trajectory generation
- Added `scripts/trajectory_generator.py`, cubic polynomial with zero-velocity boundaries.
  Verified standalone (boundary conditions exact).

## Phase 7 — Gripper
- Added `urdf/gripper.xacro`: `ee_tip` reference frame + 2 prismatic fingers.
- `ee_tip` initially had only visual+inertial (no collision) and was silently lumped into
  `end_effector` by sdformat's URDF→SDF converter (confirmed via `gz sdf -p`), breaking
  `DetachableJoint`'s `parent_link` reference. Adding a `<collision>` did NOT fix it (lumping of
  fixed-jointed links happens regardless of collision presence) — the actual fix was pointing
  `DetachableJoint`'s `parent_link` at `end_effector` (the link `ee_tip` gets lumped into; physically
  the same rigid body).
- `left_finger_joint` did not track position commands with axis `(0,-1,0)`; changing to a matching
  `(0,1,0)` axis with a mirrored (negative) limit range did not fix it either. A `<mimic>` joint was
  tried next and found unsupported by this build's physics engine (DART:
  `[Err] Physics.cc:1808 ... does not support mimic constraints`). Root cause found empirically:
  the *declaration order* in `ros2_control.xacro`/the controller YAML mattered —
  `right_finger_joint` must be listed before `left_finger_joint`; with that order both fingers
  track correctly and symmetrically.

## Phase 8 — Pick and place
- Added `scripts/pick_and_place.py` (9-step sequence, `FollowJointTrajectory` action client for
  arm + gripper, `gz topic` publish for attach/detach).
- First run: `IK failed for target (0.45, 0.0, 0.095)` — the chosen object/approach positions fell
  in the arm's unreachable inner annulus (link2=0.30m and ee_length=0.10m differ enough that
  min reach from the shoulder offset point is |0.30-0.10|=0.20m). Recomputed object/destination
  positions at radius 0.55m, verified reachable (both grasp AND +0.08m approach height) with
  `inverse_kinematics()` before use.
- Second run: object ended up nowhere near the destination. Root cause #1:
  `gz_spawn_model.launch.py`'s `-file` spawn mode defaults x/y/z to 0.0 and passes them
  explicitly to the create service, silently overriding the SDF's own `<pose>` tag — the object
  was spawning at the world origin the entire time, not at (0.55, 0, 0.015). Fixed by passing
  explicit `x`/`y`/`z` launch arguments in `demo.launch.py`.
- Third run (after spawn-pose fix): object still ended up in the wrong place, but *deterministically*
  the same wrong place both times, ruling out a timing race. Traced by manually moving the arm and
  querying `gz topic -e -t .../dynamic_pose/info` at each step: the pick object was **already
  rigidly attached to `end_effector` and moving with it before any attach message was ever
  published** — `DetachableJoint` attaches its parent/child pair at model load time, not on the
  first attach-topic message. Fixed by publishing an explicit detach as the very first action in
  `run_demo()`.
- Fourth run (after startup-detach fix): the object stayed almost exactly at its original spawn
  position through the whole sequence — the explicit mid-sequence `/gripper/attach` message did not
  visibly re-anchor the joint. Tried: repeating the one-shot `gz topic -p` publish several times
  (ruled out a subscriber-matching race - no change) and a direct `gz service .../set_pose` call
  (returned `data: true` but did not move the entity - not further resolved).
- **Status at end of session**: arm motion, IK targeting for all 9 waypoints, gripper open/close,
  and full sequencing all execute correctly and were individually verified (log + physical pose
  checks). The rigid pickup/attach step is implemented per standard practice but its reliability in
  this specific gz-sim build was not established: the object's actual transport to the destination
  was not confirmed. This is the one open item; see PROJECT_DOCUMENTATION.md §10 for suggested
  next steps (verify gz-sim/gz_ros2_control version, inspect `DetachableJoint` source for the
  attach-message handler, or fall back to a periodic `set_pose`-based "follow" implementation with
  the request format corrected).

## Phase 9 — Accuracy testing
- Added `scripts/accuracy_test.py`. First test target set was mostly unreachable (same annulus
  issue as Phase 8) - replaced with 6 targets pre-verified reachable via `inverse_kinematics()`.
- Result: 0.000000000 mm mathematical FK/IK error; 0.0000 mm Gazebo-tracked error on all 6 targets
  (commanded joint angles vs. FK of the actually-measured `/joint_states`) - both far under the
  1 mm requirement. This required raising `gz_ros2_control`'s `position_proportional_gain` from
  its default 0.1 to 500.0 in `config/manipulator_controllers.yaml`; at the default gain the
  gravity-loaded shoulder/elbow joints did not settle to sub-mm accuracy.

## Environment issues diagnosed during this session (not code bugs, but blocking symptoms)
- `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` (pre-set in this shell's environment) made FastDDS attempt
  subnet-wide multicast discovery, unreliable in WSL2; overriding to `LOCALHOST` for all launches
  and CLI calls resolved intermittent "service not found"/"rcl node's context is invalid" errors.
- Long sessions with many spawn/SIGKILL cycles accumulate stale `/dev/shm/fastrtps_*` segments and a
  stale `ros2 daemon` graph cache; both were cleared between test iterations
  (`rm -f /dev/shm/fastrtps_*`, `ros2 daemon stop && ros2 daemon start`).
- The `gz sim` GUI process was consistently using 400-540% CPU in this WSL2 environment (no GPU
  passthrough for rendering) - added a `headless:=true` launch argument to `demo.launch.py` to run
  server-only for automated testing, which is also generally good practice for CI-style verification.

## Follow-up session — Fix: object grasp/transport reliability
- Re-diagnosed with a *continuous* `gz topic -e` pose subscription (not one-shot `-n 1` reads, which
  turned out to be genuinely fresh but easy to misinterpret in isolation) run in parallel with a full
  `pick_and_place.py` execution. Traced the object's x/y/z through the entire sequence: in a failing
  run the object did not move at all (stayed at spawn pose) despite the script logging that an attach
  message was published - conclusively isolating the fault to `DetachableJoint`'s attach handling,
  not the arm motion, IK targeting, or gripper closing (all separately confirmed correct).
- A manual, step-by-step reproduction (plain `ros2 topic pub` commands, no Python script) showed the
  attach mechanism *could* work in isolation, meaning the failure mode was inconsistent/unreliable
  rather than universally broken - not something worth continuing to chase by tuning message repeats
  or timing.
- Tested `gz service .../set_pose` directly: reliably moves a genuinely free entity (verified via
  pose trace before/after); appeared to have no effect in an earlier session only because the object
  was still under an active (undetected) `DetachableJoint` constraint at the time, fighting the
  teleport - not a set_pose bug.
- **Fix implemented**: removed the `DetachableJoint` `<gazebo>` plugin block from `gripper.xacro`
  entirely. Replaced with a pose-follow mechanism in `pick_and_place.py`: a 20 Hz rclpy timer, active
  only while `self.holding` is `True`, computes the arm's current FK tool position and calls
  `/world/empty/set_pose` via the native `gz.transport13` Python bindings (`python3-gz-transport13`,
  `python3-gz-msgs10` - already present in this ROS 2 Jazzy + gz-sim 8 install, no new dependency).
  `self.holding` is set `True` right after the gripper closes and `False` right after lowering at the
  destination, before the gripper opens.
- **Verified** (two independent fresh-launch runs, object pose read directly from Gazebo, not from
  logs): object starts at spawn pose (0.55, 0.0, 0.015), stays put through approach/descend/close,
  rises smoothly to lift height during "5. lift object", moves smoothly to the destination azimuth
  during "6. move to destination", descends during "7. lower object", and settles at
  (0.4214, 0.3536, 0.0150) - matching the intended destination (0.4213, 0.3535, 0.015) to within
  ~0.1 mm - and **remains there, motionless, after release** through gripper-open, retreat, and the
  arm's return to home. Both runs produced bit-for-bit identical final object poses.

## Redesign session — Real DetachableJoint attach/release, set_pose loop removed
Re-investigated the previous session's `DetachableJoint` conclusion from scratch, since a manual
reproduction in that same session had already shown the plugin *could* work; the earlier "genuinely
unreliable" verdict was not fully re-tested before falling back to `set_pose`. This time the plugin's
actual installed source (`gz-sim8` package, not just upstream HEAD) was read, and every failure was
reproduced and root-caused individually rather than accepted at face value:

- Re-added the `DetachableJoint` `<gazebo>` plugin block to `gripper.xacro` (`parent_link=end_effector`,
  `child_model=pick_object`, `child_link=object_link`, explicit `/gripper/attach`, `/gripper/detach`,
  `/gripper/state` topics), and removed the old set_pose-timer mechanism from `pick_and_place.py`
  entirely, plus the `gz.transport13` set_pose imports that supported it.
- **Bug found #1 (motion planning, not the plugin):** a plain HOME→APPROACH joint-space interpolation
  swings `link2` within ~1 mm of the object mid-move, well before any grasp step - fixed with a
  `TRANSIT_HEIGHT` intermediate waypoint on every approach/retreat.
- **Bug found #2 (IK branch choice, not the plugin):** the IK's default "elbow up" branch places the
  physical elbow at/below ground for this arm's low targets - fixed by requesting the "elbow down"
  branch explicitly (already implemented in `kinematics.py`, just never used with a non-default value).
- **Bug found #3 (collision geometry, not the plugin):** the IK-commanded grasp point coincides with
  `end_effector`'s own solid collision cylinder tip, so descending to the object's exact center drove
  the rod into it - fixed by shortening `end_effector`'s *collision* geometry (visual/`ee_length`
  unchanged) and descending to just above the object rather than its center.
- **Bug found #4 (commanded gripper-closed value, not the plugin):** the first-draft "closed" value
  commanded a finger gap *narrower* than the object (interpenetration, not contact) - recomputed from
  the actual finger geometry.
- **Bug found #5 (this installed plugin build, confirmed real):** `DetachableJoint` does not reliably
  start detached - traced with debug logging added temporarily to a companion process
  (`scripts/gripper_bridge.py`) and to `gz sim -v 4`'s own console output, which showed
  `Attaching entity: N` firing the instant the object model finished spawning, with no attach message
  ever sent. Fixed with an unconditional detach request at node startup, before the state machine runs
  - this matches, and was previously wrongly dismissed as, this project's *very first* diagnosis of
  this exact plugin (see the Phase 8 entry above), which turned out to have been correct.
- **Bug found #6 (this installed plugin + gz_ros2_control combination, confirmed real):** even with a
  confirmed `/gripper/state=attached`, the created joint does not propagate the parent's motion to the
  object for this arm - reproduced with a raw `FollowJointTrajectory` goal sent completely outside
  `pick_and_place.py`, with up to 5 s of settle time, producing ~0.0001 m of object displacement.
  Cross-checked against gz-sim's own official `detachable_joint.sdf` example world, where the
  identical attach/detach mechanism *does* correctly move a breadcrumb attached to a DiffDrive
  vehicle's chassis. `/joint_states` reporting `effort: nan` on every arm joint mid-motion points at
  gz_ros2_control's position interface not doing physically-consistent force integration for this
  build, despite this project's own controller-yaml comment claiming otherwise.
- **Bug found #7 (consequence of #6, confirmed real):** a `/world/empty/set_pose` correction while the
  object is attached reports success but has no effect - verified directly with `gz service` calls,
  identical request succeeding while detached and silently no-op'ing while attached. DART evidently
  will not kinematically reposition a body that is currently rigidly joint-constrained.
- Given #6 and #7, and given the project's explicit "no continuous teleport loop" requirement, the
  trade-off was put to the user directly rather than resolved unilaterally; the approved fix
  (`_sync_held_object_pose()` in `pick_and_place.py`) briefly cycles detach→set_pose→re-attach once
  per completed arm move while holding (not a timer, not per `/joint_states` tick) - `DetachableJoint`
  remains the real, authoritative attach/detach state and Gazebo joint entity; the position correction
  is bounded and tied to already-planned motions completing.
- **Verified** (multiple fresh-launch runs, object pose read directly from Gazebo): object starts at
  spawn pose and stays put through approach/descend/close; travels with the gripper in discrete,
  correctly-positioned jumps through lift/transport/place; settles at the destination within ~3 cm
  after release (a small residual nudge during the return-to-home retreat, reduced but not fully
  eliminated by routing the retreat's azimuth rotation and final descent as separate moves); and does
  not resume moving once genuinely detached and the gripper has retreated.

## Correction — full trajectory/geometry revert, grasp mechanism kept minimal
User testing after the redesign session above reported two regressions: the pick-and-place object no
longer visible, and the planned trajectory substantially different from the previously working one.
Diagnosed and reverted rather than patched further:

- **Trajectory**: the redesign session's `elbow="down"` IK-branch override, `TRANSIT_HEIGHT`
  intermediate waypoints, `GRASP_DESCENT_CLEARANCE` grasp/place height offset, and the extra
  "rotate to home azimuth" retreat waypoint were all removed from `pick_and_place.py`. Every
  Cartesian target, IK branch (back to the library default), waypoint order, and per-move duration
  is now byte-for-byte the same as the original script - confirmed both by direct computation
  (`kinematics.inverse_kinematics()` on each original target, compared against the very first
  session's logged joint values) and by a live fresh-launch run whose logged joint targets matched
  that original log line for line.
- **Gripper**: `GRIPPER_CLOSED` reverted from `0.008` back to the original `0.013`, even though this
  was previously found to command finger interpenetration on the object - restoring exact original
  behavior takes priority; flagged in-code (`pick_and_place.py`) as the fix to reach for if contact-
  launch reappears, rather than silently reapplied.
- **Robot geometry**: `manipulator.xacro`'s shortened `end_effector` collision length and
  `gripper.xacro`'s removed `ee_tip` collision were both reverted to their original geometry -
  visual appearance and all dimensions (`ee_length` etc.) were never touched by either version.
- **Object spawn/visibility**: `worlds/pick_object.sdf` and `demo.launch.py`'s spawn arguments were
  compared byte-for-byte against their state at the start of the redesign session and found
  unchanged throughout - the object was confirmed present at its correct spawn pose (0.55, 0, 0.015)
  in the physics world on a fresh headless launch, both before and after this revert. The reported
  "not visible" regression was not reproduced in this (headless, no GUI) environment; the specific
  cause was not identified, since the trajectory revert alone (re-tested twice, fresh launches, full
  demo runs) showed the object correctly visible/present throughout with no separate visibility fix
  applied. If this recurs, it should be re-reported with the exact commands/environment used
  (headless vs. GUI in particular) since that is the one variable this session could not test.
- **What was kept** (the only sanctioned change, per explicit instruction): `DetachableJoint` in
  `gripper.xacro` (unchanged from the redesign session), the startup detach in `__init__`,
  `gripper_bridge.py`, and `_sync_held_object_pose()`'s detach→set_pose→re-attach cycle, called once
  per completed arm move while holding - unchanged in mechanism, just now operating on top of the
  restored, unmodified trajectory instead of the redesigned one.
- **Verified** (two fresh-launch runs post-revert): logged joint targets at every waypoint identical
  to the original session's log; object motionless through HOME/APPROACH/GRASP_POSITION/
  CLOSE_GRIPPER (no collision observed with the restored trajectory); ATTACH/DETACH confirmed via
  `/gripper/state` at every transition; object follows in discrete jumps through LIFT/TRANSPORT/
  PLACE; final position error vs. the destination was ~5.4e-7 m and ~5.5e-7 m in the two runs
  (both far under the 1 mm requirement) - notably better than the redesigned trajectory's ~3 cm
  residual, since the restored trajectory's `RETURN_HOME` no longer disturbs the placed object.
  `accuracy_test.py` re-run after all reverts: 0.0000 mm on all 6 targets, unchanged.

## GUI object-visibility investigation — root cause found, not fixable from this project's code
The previous entry's "not visible" report was reproduced this time by testing the actual GUI (not
headless): the robot performs the full pick-and-place trajectory correctly, but `pick_object` is
never seen in the 3D view. Diagnosed thoroughly using real Gazebo state, not Python logs - real
rendered screenshots via the GUI's own `/gui/screenshot` service, saved to disk and visually
inspected directly.

- **`pick_object` was confirmed to exist correctly at every layer below the GUI's render scene, on
  every single check across this entire investigation:** `gz model --list` always listed it;
  `gz model -m pick_object -p` always reported pose (0.55, 0, 0.015) exactly; the world's
  `scene/info` service always returned its full, correct definition (name, link, `box` visual
  `0.03x0.03x0.03`, red `ambient`/`diffuse` material, `lighting: true`, correct pose) - byte for
  byte matching `worlds/pick_object.sdf`. This rules out (A) "does not exist" and (C) "wrong pose"
  outright: the object is present and correctly placed in the physics engine/ECM at all times.
- **Isolated the failure to GUI rendering specifically, not spawning:** a brand-new test model,
  spawned via a raw `gz service /world/empty/create` call at a distinct location, rendered
  correctly and immediately in one test - then, in a later fresh GUI session, an identical test
  (and several more, via different mechanisms: raw service call, `ros2 run ros_gz_sim create`,
  `ros2 launch ros_gz_sim gz_spawn_model.launch.py` - the exact launch file `demo.launch.py` itself
  uses) all failed to render, exactly like `pick_object`. This shows the bug is not particular to
  `pick_object.sdf`, to the `ros_gz_sim create` spawn mechanism, or to spawn timing/ordering - it is
  an intermittent GUI-side scene-rendering fault that can affect *any* entity created after the GUI
  window is already up, in this environment.
- **Ruled out spawn timing as the cause**, despite an initial plausible lead: gz-sim's `GuiRunner`
  (`src/gui/GuiRunner.cc`) only starts applying incremental `/world/<world>/state` updates after its
  own one-shot initial-state request completes, with no retry if an update is missed while that
  request is in flight - a real, code-confirmed race window. However, neither a shorter (2.0 s,
  original) nor longer (6.0 s) spawn delay, nor spawning the object with no delay at all
  (immediately, exactly like the robot, which never has this problem), made a reliable difference -
  ruling out this specific race as the actual cause in practice, even though the code path is real.
- **Ruled out "just needs more time"**: a fresh GUI launch, left completely untouched (no camera
  moves, no service calls, no further spawns) for a full 5 minutes, still did not show the object.
  An earlier session *did* eventually show a previously-missing test object after several minutes of
  mixed interaction, but this was not reproducible as a simple function of elapsed time alone, and
  is presumed to have been coincidental with some other action, not a genuine periodic self-heal.
- **Confirmed the GUI's render loop is otherwise live and correct**: every screenshot captured
  throughout this investigation showed the robot's *existing* geometry (already part of the scene
  before the object spawn) rendered correctly and reflecting its current joint state - the render
  loop is not frozen or stalled in general. The fault is specifically in the step that adds a
  newly-created entity into the render scene (`RenderUtil::Update()`'s `newModels` queue, per
  gz-sim8's own source), independent of the main per-frame render path.
- **Root cause**: a GUI-side (`gz-sim`/`gz-rendering`) scene-synchronization fault, most likely
  aggravated by this environment's already-documented heavy software-rendering load (the `gz sim`
  GUI process runs at 400-540% CPU with no GPU passthrough in this WSL2 setup - see the Phase 9
  entry above). This is answer **(D)** from the four possibilities investigated: exists, correctly
  positioned, but a Gazebo/GUI-side issue independent of this project's spawn pipeline is hiding it.
  It was not resolved, because every angle tried (spawn timing, spawn mechanism, forced camera/view
  service calls, extra spawns as a "nudge", and a clean elapsed-time control) failed to produce a
  reliable fix, and the evidence points to a fault inside gz-sim's own GUI rendering code, outside
  this project's SDF/launch/Python files. Per explicit instruction, no further code changes were
  made chasing this specific issue once that was established; `demo.launch.py`'s object-spawn
  `TimerAction` was left at its original `period=2.0` (an earlier `period=6.0` and a no-delay attempt
  during this investigation were both reverted, since neither was proven to fix anything real).
- **Not affected**: pick-and-place functionality itself. Every check that queries Gazebo's actual
  entity/pose state directly (as opposed to what the GUI happens to be rendering) - `gz model`,
  `scene/info`, and the live `dynamic_pose/info` pose trace used throughout this project's testing -
  confirms the object exists, is grasped, travels, and settles at the destination correctly whether
  or not the GUI happens to be displaying it. Headless runs (`headless:=true`), which this project
  already recommends for reliable testing in this environment, are unaffected by this GUI-only fault.
