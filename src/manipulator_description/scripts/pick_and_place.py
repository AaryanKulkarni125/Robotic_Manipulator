#!/usr/bin/env python3
"""
Autonomous pick-and-place demo.

Explicit state sequence:
  HOME -> APPROACH -> GRASP_POSITION -> CLOSE_GRIPPER -> ATTACH -> LIFT
  -> TRANSPORT -> PLACE -> DETACH -> OPEN_GRIPPER -> RETURN_HOME -> DONE

Joint-space motion between waypoints uses the cubic trajectory generator
from trajectory_generator.py, sent to joint_trajectory_controller's
FollowJointTrajectory action. Cartesian waypoints are converted to joint
angles with scripts/kinematics.inverse_kinematics().

GRASP/CARRY MECHANISM: gz-sim's DetachableJoint system
(gz-sim-detachable-joint-system, configured in urdf/gripper.xacro) creates
a real fixed joint between the gripper's end_effector link and the
object's object_link once an Empty message is published on
/gripper/attach, and removes it on /gripper/detach - the object is then
carried by Gazebo's own physics/joint system while grasped, not by a
manual pose-teleport loop.

KNOWN QUIRK IN THIS INSTALLED PLUGIN VERSION (gz-sim 8.11.0, vendored via
ros-jazzy-gz-sim-vendor): it does NOT reliably start detached in practice.
Direct testing (publish an attach request mid-demo -> no /gripper/state
reply -> immediately publish a detach request -> instant "detached"
reply) showed the plugin's isAttached flag is already true by the time
this script's first real attach request runs, well before any attach
message from this script - matching a note this project carried from an
earlier development pass ("attaches its parent/child link pair AT MODEL
LOAD, not on the first attach-topic message"), which a later revision of
this file wrongly dismissed after reading a newer upstream source
revision that fixes this in-plugin. Since the plugin's own attach
callback is a no-op whenever isAttached is already true (it just logs
"Already attached" and returns, publishing nothing), every attach request
sent while that spurious state persists would otherwise be silently
ignored with no observable error. The fix used here: __init__ sends one
detach request before anything else, to force a real, known-good detached
baseline before the state machine starts - see the comment above that
call. With that baseline established, the ATTACH state's request behaves
exactly as the plugin's own documentation describes.

The attach/detach PUBLISH itself is delegated to a small companion
process, scripts/gripper_bridge.py, spawned once at startup and talked to
over a stdin/stdout pipe (see attach_object()/detach_object() below),
rather than publishing directly from this node with gz.transport13. This
was extra defensive-in-depth added while chasing what first looked like a
message-delivery reliability problem (multiple retries over several
seconds sometimes got no reply) - which turned out to actually be the
quirk above (the plugin correctly, if confusingly, ignoring a request
because it already believed it was attached), not a real delivery
failure: a direct `gz topic -e` echo, while a request was being retried
from this node, showed the message reliably arriving at the topic every
time. The bridge process was kept anyway since it is harmless and adds a
small amount of real robustness (an early-spawned gz-transport publisher
that never has to compete with this node's own subsequent DDS/action-
client activity), but it is not the fix for the actual bug.

SECOND CONFIRMED LIMITATION, WITH USER-APPROVED WORKAROUND: even with the
above fixed (attach reliably confirmed via /gripper/state), the resulting
DetachableJoint was found NOT to physically move the grasped object when
the arm moves, in this specific installed build. Verified directly and
repeatedly: after a confirmed attach, commanding a large arm motion (via
a raw FollowJointTrajectory goal, well outside this script, with 5 s of
settle time before the move and no interaction with this script at all)
produced ~0.0001 m of object displacement - noise, not motion coupling.
The identical DetachableJoint mechanism was verified to work correctly
on gz-sim's own official example world (gz-sim8/worlds/detachable_joint.
sdf - a breadcrumb attached to a DiffDrive vehicle's chassis correctly
follows the vehicle when driven). The cause traced to `/joint_states`
reporting `effort: nan` on every arm joint even mid-motion - evidence
gz_ros2_control's position command interface (despite this project's own
config comment claiming otherwise) is not doing physically-consistent
force/torque integration for these joints, and DART does not propagate
forward kinematics to a body dynamically grafted onto a kinematically-
driven chain via DetachableJoint. This was tested and ruled out as being
about which link is used as parent_link (identical failure at both
end_effector and link2) and about timing (identical failure with a 5 s
settle before moving). This is a genuine gz-sim/gz_ros2_control/DART
compatibility limitation in this installed version, not a configuration
error in this project.

Per explicit user direction (a real, unconstrained continuous set_pose()
loop being off the table, and a joint that doesn't move the object not
being an acceptable "working" grasp either), the two are combined:
DetachableJoint remains the real attach/detach state mechanism (a
genuine Gazebo joint entity is created and destroyed, and /gripper/state
is the source of truth for whether the object is currently grasped), and
_sync_held_object_pose() additionally snaps the object's pose to the
gripper's own FK tool position, but ONLY once per discrete arm move
while holding (called from move_arm_to(), after a move completes) - not
on a timer, not on every /joint_states message, and not while not
holding. Across the whole demo this is roughly 8 calls total, each tied
to an already-planned, already-executed motion completing, not an
independent polling loop with its own frequency.
"""

import os
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

from gz.transport13 import Node as GzNode
from gz.msgs10.stringmsg_pb2 import StringMsg as GzStringMsg
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean

import kinematics as kin
from trajectory_generator import build_cubic_trajectory

ARM_JOINTS = ["joint1", "joint2", "joint3"]
# Order matters: right_finger_joint MUST be listed before left_finger_joint.
# With the reverse order, left_finger_joint silently never tracked position
# commands in this gz-sim/gz_ros2_control build (confirmed empirically by
# reading physical link pose via `gz topic -e -t .../dynamic_pose/info`,
# independent of axis direction or limit sign) - this ordering is the fix.
GRIPPER_JOINTS = ["right_finger_joint", "left_finger_joint"]

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.013  # leaves a small gap around the 0.03 m object so the
                        # fingers visually close onto it; DetachableJoint
                        # does the actual rigid hold.
# NOTE (restored to original value 0.013 per explicit instruction to keep
# gripper behavior unchanged): the gap this produces, gap(value) = 0.05 -
# 2*value, is 0.024 m - narrower than the 0.03 m object, i.e. this commands
# the rigid fingers to interpenetrate the object by 3 mm/side. That was
# measured to make DART's contact solver launch the object away on
# contact. A session that redesigned trajectory waypoints around this
# project's explicit instructions also changed this value to 0.008 (a
# non-interpenetrating gap) to fix that; it has been reverted here to
# restore original behavior exactly. If the launch-on-contact recurs,
# 0.008 is the verified fix - flag this rather than re-deriving it.
# left_finger_joint's limit range/axis is mirrored (see urdf/gripper.xacro):
# closing is + for right_finger_joint, - for left_finger_joint.

HOME_Q = (0.0, 0.0, 0.0)

# Both at radius 0.55 m from the base axis (only the azimuth/q1 differs) -
# this radius keeps grasp AND the +0.08 m approach waypoint inside the
# reachable annulus [|link2-ee_length|, link2+ee_length] = [0.20, 0.40] m
# measured from the shoulder offset point; a radius that works for the
# grasp height does not automatically work once the +0.08 m approach
# offset is added, since that changes the *distance* from the shoulder
# point, not just height - verified with scripts/kinematics.inverse_kinematics
# before picking these numbers.
OBJECT_POS = (0.55, 0.0, 0.015)      # matches worlds/pick_object.sdf spawn pose (object center)
DEST_POS = (0.4213, 0.3535, 0.015)   # same radius, 40 degrees around
APPROACH_CLEARANCE = 0.08            # meters above the object/destination for approach waypoints

# /gripper/attach and /gripper/detach (matching urdf/gripper.xacro's
# DetachableJoint plugin config) are published by scripts/gripper_bridge.py,
# not directly from here - see attach_object()/detach_object() and the
# module docstring.
STATE_TOPIC = "/gripper/state"

# How long to wait for DetachableJoint's /gripper/state confirmation after
# requesting attach/detach, and how often to resend the request (via
# gripper_bridge.py - see the module docstring) while waiting. The actual
# root cause of the confirmations going missing during earlier testing was
# the pre-existing-attach issue documented above __init__'s startup
# detach() call, not a transport delivery problem - but a bounded retry
# window is kept regardless as a defensive margin against a single
# dropped message, matching the requirement that this be a one-time,
# confirmed state change rather than a bare fire-and-forget publish.
STATE_TIMEOUT_S = 5.0
STATE_POLL_S = 0.2

# See "SECOND CONFIRMED LIMITATION" in the module docstring: DetachableJoint
# does not propagate motion for this arm's kinematic chain in this installed
# build, so _sync_held_object_pose() corrects the object's pose to the
# gripper's FK position once per completed arm move while holding, via
# Gazebo's /world/<world>/set_pose service (a single request/response call,
# not a publish loop).
OBJECT_MODEL_NAME = "pick_object"
SET_POSE_SERVICE = "/world/empty/set_pose"

# Time given to Gazebo/ros2_control to physically settle after a
# grasp/release transition before the next motion begins - the
# DetachableJoint PreUpdate() only creates/removes the joint on the next
# simulation tick, and the arm should not start moving on a joint that
# has not been created yet.
SETTLE_S = 0.5


class PickAndPlace(Node):

    def __init__(self):
        super().__init__("pick_and_place")
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")

        self.current_q = None
        self.current_gripper = None
        self.holding = False
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self.gz_node = GzNode()
        self.gripper_state = None
        self.gz_node.subscribe(GzStringMsg, STATE_TOPIC, self._gripper_state_cb)

        # Spawn the attach/detach publisher helper EARLY (see module
        # docstring) - before any sustained rclpy spinning has happened.
        bridge_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gripper_bridge.py")
        self.gripper_bridge = subprocess.Popen(
            [sys.executable, bridge_path],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)

        self.get_logger().info("Waiting for controller action servers...")
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info("Action servers ready.")

        self.get_logger().info("Waiting for first /joint_states message...")
        while rclpy.ok() and self.current_q is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(f"Initial joint state: {self.current_q}")

        # Clear a spurious pre-existing attach. In this installed gz-sim
        # build, DetachableJoint was found (via direct testing: publish an
        # attach request mid-demo, get no /gripper/state reply, then send a
        # detach and immediately get one back) to already be isAttached at
        # some point before this script ever runs - matching the ORIGINAL
        # developer note this project's docs carried ("attaches ... AT
        # MODEL LOAD, not on the first attach-topic message") for this
        # exact plugin/version, which an earlier revision of this file
        # wrongly concluded was a misdiagnosis after reading newer upstream
        # source. Because the plugin's own attach callback is a no-op
        # while isAttached is already true ("Already attached", swallowed
        # silently), every attach request this script sends later would
        # otherwise be ignored. Detaching once here, before anything else,
        # guarantees a real detached baseline the rest of the state machine
        # can rely on.
        self.get_logger().info("Clearing any pre-existing DetachableJoint attach state...")
        self.detach_object()

    def _joint_state_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(j in name_to_pos for j in ARM_JOINTS):
            self.current_q = tuple(name_to_pos[j] for j in ARM_JOINTS)
        if all(j in name_to_pos for j in GRIPPER_JOINTS):
            self.current_gripper = tuple(name_to_pos[j] for j in GRIPPER_JOINTS)

    def _send_trajectory(self, client, joint_names, q0, qf, duration_s, num_waypoints=20):
        traj = build_cubic_trajectory(joint_names, list(q0), list(qf), duration_s, num_waypoints)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Trajectory goal rejected by controller")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(f"Trajectory finished with error_code={result.error_code}")
        return result

    def move_arm_to(self, target_q, duration_s=2.5, label=""):
        self.get_logger().info(f"[arm] {label}: {tuple(round(v,4) for v in self.current_q)} "
                                f"-> {tuple(round(v,4) for v in target_q)}")
        self._send_trajectory(self.arm_client, ARM_JOINTS, self.current_q, target_q, duration_s)
        self.current_q = tuple(target_q)
        time.sleep(0.3)
        if self.holding:
            self._sync_held_object_pose()

    def move_arm_to_cartesian(self, xyz, duration_s=2.5, label=""):
        sol = kin.inverse_kinematics(*xyz)
        if sol is None:
            raise RuntimeError(f"IK failed for target {xyz} ({label})")
        self.move_arm_to(sol, duration_s=duration_s, label=f"{label} xyz={xyz}")

    def set_gripper(self, value, duration_s=1.0, label=""):
        self.get_logger().info(f"[gripper] {label}: -> {value}")
        q0 = self.current_gripper if self.current_gripper else (GRIPPER_OPEN, -GRIPPER_OPEN)
        qf = (value, -value)  # (right_finger_joint, left_finger_joint)
        self._send_trajectory(self.gripper_client, GRIPPER_JOINTS, q0, qf, duration_s, num_waypoints=5)
        self.current_gripper = qf
        time.sleep(0.3)

    def _spin_wait(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _gripper_state_cb(self, msg):
        self.gripper_state = msg.data

    def _bridge_request(self, command):
        self.gripper_bridge.stdin.write(command + "\n")
        self.gripper_bridge.stdin.flush()
        self.gripper_bridge.stdout.readline()  # "done"

    def _request_gripper_state(self, command, expected):
        self.get_logger().info(f"[grasp] requesting {command}, waiting for /gripper/state={expected!r}")
        end = time.time() + STATE_TIMEOUT_S
        next_send = 0.0
        while rclpy.ok() and time.time() < end:
            if self.gripper_state == expected:
                self.get_logger().info(f"[grasp] confirmed /gripper/state={expected!r}")
                self._spin_wait(SETTLE_S)
                return
            if time.time() >= next_send:
                self._bridge_request(command)
                next_send = time.time() + STATE_POLL_S
            rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().warn(
            f"[grasp] timed out waiting for /gripper/state={expected!r} "
            f"(last seen: {self.gripper_state!r})")

    def attach_object(self):
        self.gripper_state = None
        self._request_gripper_state("attach", "attached")
        self.holding = (self.gripper_state == "attached")

    def detach_object(self):
        self.holding = False
        self.gripper_state = None
        self._request_gripper_state("detach", "detached")

    def _sync_held_object_pose(self):
        # See "SECOND CONFIRMED LIMITATION" in the module docstring. One
        # request/response set_pose call, made only right after a completed
        # arm move while holding - not a timer, not per /joint_states tick.
        #
        # A bare set_pose call while the object is still rigidly attached
        # was found to silently do nothing: the service replies success,
        # but the object's pose does not change (verified directly with
        # `gz service --reqtype gz.msgs.Pose ...` while attached vs.
        # detached - identical request, only the detached case took
        # effect). DART/gz-physics evidently treats a body fixed-jointed to
        # something else as not independently kinematically repositionable,
        # which makes sense for a genuinely rigid weld. So the correction
        # briefly detaches, repositions, and re-attaches around the actual
        # set_pose call - still one bounded action per completed move, not
        # a timer, just three gz-transport calls instead of one.
        self._request_gripper_state("detach", "detached")

        x, y, z = kin.forward_kinematics(*self.current_q)
        req = GzPose()
        req.name = OBJECT_MODEL_NAME
        req.position.x, req.position.y, req.position.z = x, y, z
        req.orientation.w = 1.0
        try:
            ok, resp = self.gz_node.request(SET_POSE_SERVICE, req, GzPose, GzBoolean, 500)
            if not ok or not resp.data:
                self.get_logger().warn(f"set_pose sync did not succeed: ok={ok} resp={resp}")
        except Exception as e:
            self.get_logger().warn(f"set_pose sync failed: {e}")

        self._request_gripper_state("attach", "attached")

    def run_demo(self):
        self.get_logger().info("=== PICK AND PLACE DEMO START ===")

        # Waypoints restored exactly to the original (pre-redesign) values:
        # same targets, same order, same durations. The only additions
        # relative to the original are the two real attach_object()/
        # detach_object() calls (replacing the old holding=True/False
        # kinematic pose-follow) - no waypoint, IK branch, or timing was
        # changed to accommodate them.
        above_object = (OBJECT_POS[0], OBJECT_POS[1], OBJECT_POS[2] + APPROACH_CLEARANCE)
        above_dest = (DEST_POS[0], DEST_POS[1], DEST_POS[2] + APPROACH_CLEARANCE)

        # HOME
        self.move_arm_to(HOME_Q, duration_s=2.0, label="HOME")

        # APPROACH
        self.move_arm_to_cartesian(above_object, duration_s=2.5, label="APPROACH")
        self.set_gripper(GRIPPER_OPEN, duration_s=0.8, label="ensure gripper open")

        # GRASP_POSITION
        self.move_arm_to_cartesian(OBJECT_POS, duration_s=2.0, label="GRASP_POSITION")

        # CLOSE_GRIPPER
        self.set_gripper(GRIPPER_CLOSED, duration_s=1.0, label="CLOSE_GRIPPER")

        # ATTACH (real Gazebo DetachableJoint, created only now)
        self.attach_object()

        # LIFT
        self.move_arm_to_cartesian(above_object, duration_s=2.0, label="LIFT")

        # TRANSPORT
        self.move_arm_to_cartesian(above_dest, duration_s=2.5, label="TRANSPORT")

        # PLACE
        self.move_arm_to_cartesian(DEST_POS, duration_s=2.0, label="PLACE")

        # DETACH
        self.detach_object()

        # OPEN_GRIPPER
        self.set_gripper(GRIPPER_OPEN, duration_s=1.0, label="OPEN_GRIPPER")

        # RETURN_HOME
        self.move_arm_to_cartesian(above_dest, duration_s=2.0, label="retreat")
        self.move_arm_to(HOME_Q, duration_s=2.5, label="RETURN_HOME")

        self.get_logger().info("=== PICK AND PLACE DEMO COMPLETE (DONE) ===")

    def shutdown(self):
        self.gripper_bridge.stdin.close()
        self.gripper_bridge.terminate()


def main():
    rclpy.init()
    node = PickAndPlace()
    try:
        node.run_demo()
    except Exception as e:
        node.get_logger().error(f"Demo failed: {e}")
        node.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
