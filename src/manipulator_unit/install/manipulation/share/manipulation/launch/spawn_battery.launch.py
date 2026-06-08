from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory
import random
import math
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('ros_industrial_support')
    model_path = pkg_path + '/models/battery/model.sdf'

    entity_name = 'battery_' + str(random.randint(0, 1000))
    
    # ignition gazebo spawn entity node
    gazebo_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-name', entity_name,
            '-file', model_path,
            '-x', '0.4', '-y', '-0.4', '-z', '1.5', '-P', str(math.radians(180)),
        ],
        #parameters=[{'use_sim_time': True}],
    )


    return LaunchDescription([gazebo_spawn_entity_node])
