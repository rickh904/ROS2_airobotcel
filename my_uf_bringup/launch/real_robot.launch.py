#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.164')
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='lite')
    report_type = LaunchConfiguration('report_type', default='normal')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    attach_to = LaunchConfiguration('attach_to', default='xarm_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
   
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotSystemHardware'

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('my_uf_moveit_config'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    pkg_path = os.path.join(get_package_share_directory('my_uf_moveit_config'))
    bringup_pkg_path = get_package_share_directory('my_uf_bringup')
    urdf_file = os.path.join(pkg_path, 'config', 'uf_robot.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'config', 'uf_robot.srdf')
    extra_robot_api_params_path = os.path.join(bringup_pkg_path, 'config', 'xarm_api_params.yaml')

    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            robot_ip=robot_ip,
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
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .trajectory_execution(file_path=controllers_file)
        .planning_pipelines(config_folder=pipeline_filedir)
        .to_moveit_configs()
    )
    
    # robot description launch
    # xarm_description/launch/_robot_description.launch.py
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('my_uf_description'), 'launch', '_robot_description.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
        }.items(),
    )

    # robot moveit common launch
    # xarm_moveit_config/launch/_robot_moveit_common2.launch.py
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common2.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'use_sim_time': 'false',
            'moveit_config_dump': yaml.dump(moveit_config.to_dict()),
            'rviz_config': PathJoinSubstitution([FindPackageShare('my_uf_bringup'), 'rviz', 'moveit.rviz'])
        }.items(),
    )

    # joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'source_list': ['{}{}/joint_states'.format(prefix.perform(context), hw_ns.perform(context))]}],
        remappings=[
            ('follow_joint_trajectory', '{}uf_traj_controller/follow_joint_trajectory'.format(prefix.perform(context))),
        ],
    )

    # ros2 control launch
    # xarm_controller/launch/_ros2_control.launch.py
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_ros2_control.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
            'ros2_control_params': ros2_control_params,
        }.items(),
    )

    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'dof': dof,
            'hw_ns': hw_ns,
            'prefix': prefix,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'add_bio_gripper': add_bio_gripper,
            'robot_type': robot_type,
            'extra_robot_api_params_path': extra_robot_api_params_path,
        }.items(),
    )

    controllers = [
        'joint_state_broadcaster',
        '{}uf_traj_controller'.format(prefix.perform(context)),
    ]
    # Load controllers
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    # control_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     output='screen',
    #     arguments=[
    #         '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
    #         '--controller-manager', '{}/controller_manager'.format(ros_namespace)
    #     ],
    # )

    static_traansfer_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_traansfer_frame',
        output='screen',
        arguments=['-0.025', '0.10', '0.05', '0.0', '0.0', '0.0', 'world', 'test_transfer_frame'],
    )


    return [
        robot_description_launch,
        robot_moveit_common_launch,
        joint_state_publisher_node,
        ros2_control_launch,
        robot_driver_launch,
        static_traansfer_frame,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
