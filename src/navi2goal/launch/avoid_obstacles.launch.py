from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_to_object',
            executable='getObjectRange'
        ),
        Node(
            package='navigation_to_object',
            executable='goToGoal'
        )
    ])
