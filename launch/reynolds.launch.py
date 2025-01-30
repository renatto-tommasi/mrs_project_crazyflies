# reynolds.launch.py
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # # Command-line argument for number of robots
    # num_of_robots_launch_arg = DeclareLaunchArgument(
    #     "num_of_robots", default_value=TextSubstitution(text="3")
    # )

    # # Get the number of robots from the launch configuration
    # num_of_robots = LaunchConfiguration('num_of_robots')

    # Create multiple boid nodes
    boid_nodes = [
        Node(
            package='mrs_project_crazyflies',
            namespace=f'boid{i}',
            executable='reynolds',
            name='reynolds_sim',
            parameters=[{
                "num_of_robots": int(os.environ.get('NUM_ROBOTS', '3')), 
                "robot_id": i,
            }]
        ) for i in range(1, int(os.environ.get('NUM_ROBOTS', '3')) + 1)
    ]

    return LaunchDescription(boid_nodes)