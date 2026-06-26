import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # 1. Robot IP-adres argument
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.173',
        description='IP-adres van de Lite6 robot'
    )

    robot_ip = LaunchConfiguration('robot_ip')

    # 2. Robot + MoveIt bringup
    my_uf_bringup_dir = get_package_share_directory('my_uf_bringup')

    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                my_uf_bringup_dir,
                'launch',
                'real_robot.launch.py'
            )
        ),
        launch_arguments={
            'robot_ip': robot_ip
        }.items()
    )

    # 3. Vision launch, maar NIET meteen starten
    ai_vision_dir = get_package_share_directory('ai_vision')

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ai_vision_dir,
                'launch',
                'vision.launch.py'
            )
        )
    )

    # 4. Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    uf_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "uf_traj_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    delayed_controller_spawners = TimerAction(
        period=8.0,
        actions=[
            joint_state_broadcaster_spawner,
            uf_traj_controller_spawner
        ]
    )

    # 5. Robot.py en transformatie starten nadat controllers tijd hebben gehad
    robot_node = Node(
        package='manipulator_unit',
        executable='Robot',
        name='robot_manipulator',
        output='screen'
    )

    transformatie_node = Node(
        package='transformatie_pakket',
        executable='positie_transformatie.py',
        name='positie_transformatie',
        output='screen'
    )

    delayed_robot_nodes = TimerAction(
        period=22.0,
        actions=[
            robot_node,
            transformatie_node
        ]
    )

    # 6. Vision pas starten als robot bringup rustiger is
    delayed_vision = TimerAction(
        period=30.0,
        actions=[
            vision_launch
        ]
    )

    # 7. Main en HMI als laatste starten
    # Main wacht dan netjes op:
    # - /ai_vision/coord_ref
    # - Coord_Robot
    # - manipulator_task
    main_controller_node = Node(
        package='controller',
        executable='main_controller',
        name='main_controller',
        output='screen'
    )

    hmi_node = Node(
        package='hmi_unit',
        executable='hmi_node',
        name='hmi_unit',
        output='screen'
    )

    delayed_main_hmi_nodes = TimerAction(
        period=40.0,
        actions=[
            main_controller_node,
            hmi_node
        ]
    )

    return LaunchDescription([
        robot_ip_arg,

        # Eerst robot + MoveIt
        real_robot_launch,

        # Dan controllers
        delayed_controller_spawners,

        # Dan Robot.py + transformatie
        delayed_robot_nodes,

        # Dan vision
        delayed_vision,

        # Als laatste main + HMI
        delayed_main_hmi_nodes
    ])