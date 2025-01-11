# reynolds.launch.py

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # Command-line argument for number of robots
    num_of_robots_launch_arg = DeclareLaunchArgument(
        "num_of_robots", default_value=TextSubstitution(text="4")
    )

    # Create multiple boid nodes
    boid_nodes = [
        Node(
            package='mrs_project',
            namespace=f'boid{i+1}',
            executable='reynolds',
            name='reynolds_sim',
            parameters=[{
                "num_of_robots": LaunchConfiguration('num_of_robots'),
                "robot_id": i+1,
            }]
        ) for i in range(4)
    ]

    return LaunchDescription([num_of_robots_launch_arg] + boid_nodes)
