
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_navigation',
            executable='image_recog_knn',
            name='image_recognition_node',
            output='screen'
        ),

        Node(
            package='maze_navigation',
            executable='getObjectRange',
            name='getObjectRange_node',
            output='screen'
        ),

        Node(
            package='maze_navigation',
            executable='goToGoal',
            name='goToGoal_node',
            output='screen'
        ),
    ])
