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
import math
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import load_python_launch_file_as_module
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.actions import OpaqueFunction
from launch_param_builder import load_xacro
from launch.actions import ExecuteProcess

    
def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    show_rviz = LaunchConfiguration('show_rviz', default=False)

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    rviz_config = LaunchConfiguration('rviz_config', default='')
    moveit_config_dump = LaunchConfiguration('moveit_config_dump', default='')
    load_controller = LaunchConfiguration('load_controller', default=True)

    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader) if moveit_config_dump else {}
    moveit_config_package_name = 'my_uf_bringup'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    robot_description = {'robot_description': moveit_config_dict['robot_description']}
    robot_description_content = moveit_config_dict.get('robot_description', '')

    # robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
 
    # ignition gazebo launch
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('my_uf_bringup'), 'worlds', 'empty_world.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ' -r -v 4 {}'.format(xarm_gazebo_world.perform(context)),
        }.items(),
    )

    # ignition gazebo spawn entity node
    robot_description_content = moveit_config_dict.get('robot_description', '')
    
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'uf_robot',
            '-string', robot_description_content,
            '-x', '0',
            '-y', '0',
            '-z', '0.01',
        ],
        parameters=[{'use_sim_time': True}],
    )


    # rviz with moveit configuration
    if not rviz_config.perform(context):
        rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'environment.rviz'])
    else:
        rviz_config_file = rviz_config
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config_dict.get('robot_description', ''),
                'robot_description_semantic': moveit_config_dict.get('robot_description_semantic', ''),
                'robot_description_kinematics': moveit_config_dict.get('robot_description_kinematics', {}),
                'robot_description_planning': moveit_config_dict.get('robot_description_planning', {}),
                'planning_pipelines': moveit_config_dict.get('planning_pipelines', {}),
                'use_sim_time': True
            }
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )
    # Load controllers
    controllers = [
        'joint_state_broadcaster',
        '{}uf_traj_controller'.format(prefix.perform(context)),
    ]
    if robot_type.perform(context) != 'lite' and add_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}{}_gripper_traj_controller'.format(prefix.perform(context), robot_type.perform(context)))
    elif robot_type.perform(context) != 'lite' and add_bio_gripper.perform(context) in ('True', 'true'):
        controllers.append('{}bio_gripper_traj_controller'.format(prefix.perform(context)))
    
    controller_nodes = []
    if load_controller.perform(context) in ('True', 'true'):
        for controller in controllers:
            controller_nodes.append(Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=[
                    controller,
                    '--controller-manager', '{}/controller_manager'.format(ros_namespace)
                ],
                parameters=[{'use_sim_time': True}],
            ))

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])

    # NOTE: We do NOT publish robot_description as a topic for Gazebo simulations
    # The gz_ros2_control plugin creates its own controller_manager from the URDF
    # and publishing robot_description causes a conflict/segfault

    # Build the list of nodes to launch
    nodes_to_launch = [
        robot_state_publisher_node,
        clock_bridge,
        # robot_description_publisher_node,  # Commented out for Gazebo
        gazebo_launch,
        spawn_robot_node,
    ]
    
    # Add RViz if requested
    if 0:
        if show_rviz.perform(context) in ('True', 'true'):
            nodes_to_launch.append(rviz2_node)
    
    # Add controllers if requested
    if len(controller_nodes) > 0:
        nodes_to_launch.extend(controller_nodes)
    
    return nodes_to_launch


def generate_launch_description():
    # Set Gazebo resource path to find xarm_description meshes
    # For model://xarm_description/meshes/... to work, we need the parent of xarm_description
    xarm_description_path = get_package_share_directory('xarm_description')
    # Get parent directory (remove /xarm_description from the end)
    xarm_share_parent = os.path.dirname(xarm_description_path)
    
    # Add ROS2 lib directory to Gazebo plugin path to find gz_ros2_control-system plugin
    gz_plugin_path = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    ros2_lib_path = '/opt/ros/jazzy/lib'
    if ros2_lib_path not in gz_plugin_path:
        if gz_plugin_path:
            gz_plugin_path = f"{ros2_lib_path}:{gz_plugin_path}"
        else:
            gz_plugin_path = ros2_lib_path
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=xarm_share_parent
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=gz_plugin_path
        ),
        OpaqueFunction(function=launch_setup)
    ])
