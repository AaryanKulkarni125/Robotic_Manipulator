import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'manipulator_description'
    package_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(package_share, 'urdf', 'manipulator.xacro')
    object_sdf = os.path.join(package_share, 'worlds', 'pick_object.sdf')

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    headless = LaunchConfiguration('headless')

    # A single spawner process loading all three controllers sequentially -
    # running multiple concurrent spawner processes against one
    # controller_manager was observed to cause lock contention/service
    # discovery flakiness. One process, one at a time, is reliable.
    controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'joint_trajectory_controller', 'gripper_controller'],
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run gz sim server-only (-s), no GUI. Useful for automated tests / low-GPU environments.'
        ),

        # Gazebo Sim, built-in empty world (ground plane + sun)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items(),
            condition=UnlessCondition(headless)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-s -r empty.sdf'}.items(),
            condition=IfCondition(headless)
        ),

        # robot_state_publisher (xacro is the single source of truth)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        Command(['xacro ', xacro_file]), value_type=str),
                    'use_sim_time': True
                }
            ]
        ),

        # Spawn the robot (arm + gripper) immediately, synchronously - matching
        # the sequencing of gazebo.launch.py (Phase 3), which is known-good.
        # The gz_ros2_control plugin (ros2_control.xacro) starts
        # controller_manager inside the gz sim process at this point.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
            ),
            launch_arguments={
                'topic': '/robot_description',
                'entity_name': 'manipulator'
            }.items()
        ),

        # Spawn the pick object after the robot.
        # gz_spawn_model.launch.py's 'create' call defaults x/y/z to 0.0 and
        # passes them explicitly, which OVERRIDES the SDF file's own <pose>
        # tag rather than falling back to it - the object was silently
        # spawning at the world origin until this was made explicit here.
        # These values must match OBJECT_POS in scripts/pick_and_place.py
        # and the <pose> in worlds/pick_object.sdf (kept in sync manually;
        # the <pose> tag itself is now only a documentation fallback).
        #
        # NOTE on a GUI-only (non-headless) rendering issue investigated and
        # NOT fixed here: with the GUI enabled, `pick_object` was found to
        # not visually render in the 3D view, even though it is correctly
        # present in the physics engine/ECM the entire time (verified
        # repeatedly via `gz model --list`, `gz model -p pick_object`, and
        # the world's own `scene/info` service, all of which always
        # reported the correct name, pose, geometry, and material) and
        # pick-and-place works correctly against it (headless, and via
        # direct Gazebo entity/pose queries during a GUI run). Both a
        # shorter (2.0 s, original) and longer (6.0 s) spawn delay, and
        # spawning with no delay at all (immediately, like the robot),
        # were tried and made no reliable difference; a 5-minute wait with
        # zero further interaction also did not resolve it. This points to
        # a GUI-side rendering/scene-sync stall specific to this system's
        # heavily CPU-constrained software rendering (already documented
        # in docs/CHANGELOG.md as running the `gz sim` GUI process at
        # 400-540% CPU with no GPU passthrough), not a spawn-timing or
        # spawn-mechanism bug - so the period here is left at its original,
        # already-working value rather than changed on an unverified guess.
        # See docs/CHANGELOG.md's "GUI object-visibility investigation"
        # entry for the full diagnostic trail.
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
                    ),
                    launch_arguments={
                        'file': object_sdf,
                        'entity_name': 'pick_object',
                        'x': '0.55',
                        'y': '0.0',
                        'z': '0.015'
                    }.items()
                )
            ]
        ),

        # Controllers, after spawn + controller_manager startup.
        TimerAction(period=5.0, actions=[controllers_spawner]),
    ])
