# reynolds.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Command-line argument for number of robots
    num_of_robots_launch_arg = DeclareLaunchArgument(
        "num_of_robots", default_value=TextSubstitution(text="3")
    )

    # Get the number of robots from the launch configuration
    num_of_robots = LaunchConfiguration('num_of_robots')

    # Create multiple boid nodes
    boid_nodes = [
        Node(
            package='mrs_project_crazyflies',
            namespace=f'boid{i+1}',
            executable='reynolds',
            name='reynolds_sim',
            parameters=[{
                "num_of_robots": num_of_robots, 
                "robot_id": i+1,
            }]
        ) for i in range(int(num_of_robots.perform(None)))
    ]

    return LaunchDescription([num_of_robots_launch_arg] + boid_nodes)