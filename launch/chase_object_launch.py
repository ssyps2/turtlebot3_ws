from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='v4l2_camera',
        #     executable='v4l2_camera_node',
        #     parameters=['./v4l2_camera.yaml']
        # ),
        Node(
            package='object_tracker',
            executable='detect_object'
        ),
        Node(
            package='object_tracker',
            executable='get_object_range'
        ),
        Node(
            package='object_tracker',
            executable='chase_object'
        )
    ])