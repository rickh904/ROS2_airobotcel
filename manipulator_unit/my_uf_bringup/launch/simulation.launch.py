#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
# Adapted for Avans ROS2 Industrial Workshop by Gerard Harkema, may 2025

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    attach_to = LaunchConfiguration('attach_to', default='xarm_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0.0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = LaunchConfiguration('ros2_control_plugin', default='gz_ros2_control/GazeboSimSystem')

    # Create parameters file for Gazebo WITHOUT robot_description to avoid segfault
    # The gz_ros2_control plugin gets robot_description from the spawned URDF
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('my_uf_moveit_config'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type.perform(context)
    )

    pkg_path = os.path.join(get_package_share_directory('my_uf_moveit_config'))
    urdf_file = os.path.join(pkg_path, 'config', 'uf_robot.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'config', 'uf_robot.srdf')

    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
        )
        .robot_description(
            file_path=urdf_file, 
            mappings={
                'ros2_control_plugin': 'gz_ros2_control/GazeboSimSystem',
                'ros2_control_params': ros2_control_params
            }
        )
        #.robot_description_semantic(file_path=srdf_file)
        #.robot_description_kinematics(file_path=kinematics_file)
        #.joint_limits(file_path=joint_limits_file)
        #.trajectory_execution(file_path=controllers_file)
        #.planning_pipelines(config_folder=pipeline_filedir)
        .to_moveit_configs()
    )

    moveit_config_dump = yaml.dump(moveit_config.to_dict())

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('my_uf_bringup'), 'launch', 'support', '_robot_moveit_common2.launch.py'])),
    launch_arguments={
            'prefix': prefix,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'show_rviz': 'false',
            'use_sim_time': 'true',
            'moveit_config_dump': moveit_config_dump,
            'rviz_config': PathJoinSubstitution([FindPackageShare('my_uf_bringup'), 'rviz', 'environment.rviz'])
        }.items(),
    )

    # robot gazebo launch
    # mbot_demo/launch/_robot_on_mbot_gazebo.launch.py
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('my_uf_bringup'), 'launch', 'support', '_gazebo_ign.launch.py'])),
        launch_arguments={
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'moveit_config_dump': moveit_config_dump,
            'show_rviz': 'true',
            'rviz_config': PathJoinSubstitution([FindPackageShare('my_uf_bringup'),'rviz', 'environment.rviz'])
        }.items(),
    )

    # move_group launch
    movegroup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('my_uf_bringup'), 'launch', 'movegroup.launch.py'])
        ),
        launch_arguments={
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'launch_rviz': 'true',
        }.items(),
    )


    return [
        robot_gazebo_launch,
        movegroup_launch,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])