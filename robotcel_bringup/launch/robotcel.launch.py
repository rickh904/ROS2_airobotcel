import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    # 2. Includeren van de MoveIt / RViz bringup
    my_uf_bringup_dir = get_package_share_directory('my_uf_bringup')
    real_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_uf_bringup_dir, 'launch', 'real_robot.launch.py')
        ),
        launch_arguments={'robot_ip': robot_ip}.items()
    )

    # 3. CONTROLLER SPAWNERS (Dwingt activering van de hardware interfaces af)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    uf_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["uf_traj_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # 4. Definieer alle applicatie nodes
    main_controller_node = Node(
        package='controller',
        executable='main_controller',
        name='main_controller',
        output='screen'
    )

    robot_node = Node(
        package='manipulator_unit',
        executable='Robot',
        name='robot_manipulator',
        output='screen'
    )

    gohome_node = Node(
        package='manipulator_unit',
        executable='GoHome',
        name='go_home_init',
        output='screen'
    )

    hmi_node = Node(
        package='hmi_unit',
        executable='hmi_node',
        name='hmi_unit',
        output='screen'
    )

    transformatie_node = Node(
        package='transformatie_pakket',
        executable='positie_transformatie.py',
        name='positie_transformatie',
        output='screen'
    )

    # 5. Voeg alles samen
    return LaunchDescription([
        robot_ip_arg,
        real_robot_launch,
        joint_state_broadcaster_spawner,
        uf_traj_controller_spawner,
        main_controller_node,
        robot_node,
        gohome_node,
        hmi_node,
        transformatie_node
    ])