import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'manipulator_description'
    package_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(package_share, 'urdf', 'manipulator.xacro')

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    # A single spawner process loading both controllers sequentially -
    # running multiple concurrent spawner processes against one
    # controller_manager was observed to cause lock contention/service
    # discovery flakiness. One process, one at a time, is reliable.
    controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([

        # Start Gazebo Sim with the built-in empty world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),

        # Process Xacro and publish /robot_description (also feeds Gazebo spawner)
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

        # Spawn the robot into the running Gazebo world from /robot_description.
        # The gz_ros2_control plugin declared in ros2_control.xacro starts the
        # controller_manager node inside the gz sim process at this point.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
            ),
            launch_arguments={
                'topic': '/robot_description',
                'entity_name': 'manipulator'
            }.items()
        ),

        # Give the entity spawn + controller_manager startup time to finish,
        # then load controllers. A fixed delay is used (not an event handler)
        # because the spawner above runs inside an IncludeLaunchDescription
        # and its process action is not directly accessible here.
        TimerAction(
            period=5.0,
            actions=[controllers_spawner]
        ),
    ])
