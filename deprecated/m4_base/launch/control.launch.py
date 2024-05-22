from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='m4_base',
            namespace='m4_base',
            executable='tilt_controller',
            name='tilt_controller'
        ),
        Node(
            package='m4_base',
            namespace='m4_base',
            executable='drive_controller',
            name='drive_controller'
        ),
    ])
