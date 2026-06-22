#!/usr/bin/env python3
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('my_uf_moveit_config'))
    urdf_file = os.path.join(pkg_path, 'config', 'uf_robot.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'config', 'uf_robot.srdf')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')

    moveit_config = (
        MoveItConfigsBuilder(
            context=None,
            dof=6,
            robot_type='lite',
            prefix='',
            limited=True,
        )
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .to_moveit_configs()
    )

    # Include move_group launch file from my_uf_moveit_config
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_uf_moveit_config'), 'launch', 'move_group.launch.py'])
        ),
    )

    # Demo node - delayed to allow MoveIt2 to start
    demo_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='my_uf_demo_cpp',
                executable='demo',
                name='my_uf_demo_cpp',
                output='screen',
                parameters=[moveit_config.to_dict()],
            )
        ]
    )

    return LaunchDescription([
        move_group_launch,
        demo_node,
    ])
