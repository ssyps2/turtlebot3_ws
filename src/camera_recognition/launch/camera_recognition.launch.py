
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Uncomment and modify if needed for v4l2_camera
        # ExecuteProcess(
        #     cmd=['gnome-terminal', '--', 'ros2', 'run', 'v4l2_camera', 'v4l2_camera_node', '--ros-args', '--params-file', './v4l2_camera.yaml'],
        #     output='screen'
        # ),

        # Launch `color_track_server` in its own terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'camera_recognition', 'color_track_server'],
            output='screen'
        ),

        # Launch `client_test` in its own terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'camera_recognition', 'client_test'],
            output='screen'
        ),
    ])