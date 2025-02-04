# reynolds.launch.py

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    boid_nodes = [
        Node(
            package='mrs_project_crazyflies',
            namespace=f'boid{i}',
            executable='reynolds',
            name='reynolds_sim',
            parameters=[{
                "num_of_robots": int(os.environ.get('NUM_ROBOTS', '4')), 
                "robot_id": i,
            }]
        ) for i in range(1, int(os.environ.get('NUM_ROBOTS', '4')) + 1)
    ]

    return LaunchDescription(boid_nodes)
