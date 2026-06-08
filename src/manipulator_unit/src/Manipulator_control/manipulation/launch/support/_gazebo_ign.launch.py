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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    moveit_config_package_name = 'manipulation'
    xarm_type = '{}{}'.format(robot_type.perform(context), dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else '')
    
    robot_description = {'robot_description': moveit_config_dict['robot_description']}

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
 
    manipulation_share_dir = get_package_share_directory('manipulation')
    support_share_dir = get_package_share_directory('ros_industrial_support')
    resource_paths = [
        os.path.join(support_share_dir, 'models'),
        os.path.join(manipulation_share_dir, 'models'),
        os.path.join(manipulation_share_dir, 'worlds'),
    ]
    resource_path_value = os.pathsep.join(resource_paths)

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            resource_path_value,
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ]),
    )
    ign_gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.pathsep.join([
            resource_path_value,
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ]),
    )

    # ignition gazebo launch
    xarm_gazebo_world = PathJoinSubstitution([FindPackageShare('manipulation'), 'worlds', 'casus.world'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ' -r -v 4 {}'.format(xarm_gazebo_world.perform(context)),
        }.items(),
    )

    # ignition gazebo spawn entity node

    pkg_path = os.path.join(manipulation_share_dir)
    robot_on_pedestal_sdf_file = os.path.join(pkg_path, 'urdf', 'robot_on_pedestal.sdf')

    robot_on_pedestal_launch = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-name', 'xarm',
            #'-topic', robot_on_pedestal_description,
            '-file', robot_on_pedestal_sdf_file,
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
        '{}{}_traj_controller'.format(prefix.perform(context), xarm_type),
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

    # Spawn vacuum gripper
    vacuum_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_industrial_actuators'), 'launch', 'spawn_vacuum_gripper.launch.py'])),
        launch_arguments={
        }.items(),
    )

    pkg_path = support_share_dir
    model_path = pkg_path + '/models/computer_mobile/model.sdf'
    
    # ignition gazebo spawn entity node
    mobile_computer_launch = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-name', "computer_mobile",
            '-file', model_path,
            '-x', '1.5', '-y', '-0.5', '-z', '0.0', '-Y', str(math.radians(45)),
        ],
    )

    model_path = pkg_path + '/models/assembly_station/model.sdf'
    # ignition gazebo spawn entity node
    assembly_station_launch = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-name', "assembly_station",
            '-file', model_path,
            '-x', '0.5', '-y', '-0.5', '-z', '0.0', '-Y', str(math.radians(90)),
        ],
    )


    model_path = pkg_path + '/models/drop_bin/model.sdf'
    # ignition gazebo spawn entity node
    drop_bin_launch = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-name', "drop_bin",
            '-file', model_path,
            '-x', '-0.5', '-y', '0.5', '-z', '0.0',
        ],
    )

    set_cam_pose = ExecuteProcess(
            cmd=[
                'gz', 'service',
                '-s', '/gui/move_to/pose',
                '--reqtype', 'gz.msgs.GUICamera',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--req', 
                'pose: {position: {x: 3.0, y: 2.0, z: 3.0} orientation: {x: 0.261282, y: 0.1065307, z: -0.8883635, w: 0.3622062}}',
            ],
            output='screen'
        )

    if len(controller_nodes) > 0:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            gz_sim_resource_path,
            ign_gazebo_resource_path,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=robot_on_pedestal_launch,
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=robot_on_pedestal_launch,
                    on_exit=rviz2_node,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_on_pedestal_launch,
                    on_exit=controller_nodes,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_on_pedestal_launch,
                    on_exit=vacuum_gripper_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_on_pedestal_launch,
                    on_exit=mobile_computer_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_on_pedestal_launch,
                    on_exit=assembly_station_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=robot_on_pedestal_launch,
                    on_exit=drop_bin_launch,
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=drop_bin_launch,
                    on_exit=set_cam_pose,
                )
            ),
            robot_state_publisher_node,
            clock_bridge,
            
        ]
    else:
        return [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=gazebo_launch,
                )
            ),
            gz_sim_resource_path,
            ign_gazebo_resource_path,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_state_publisher_node,
                    on_start=robot_on_pedestal_launch,
                )
            ),
            RegisterEventHandler(
                condition=IfCondition(show_rviz),
                event_handler=OnProcessExit(
                    target_action=robot_on_pedestal_launch,
                    on_exit=rviz2_node,
                )
            ),
            robot_state_publisher_node
        ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
