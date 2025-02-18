from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='morphing_lander',
            executable='rl_controller_sim',
            name='rl_controller_sim',
            output='screen',    
        ),
        Node(
            package='morphing_lander',
            executable='tilt_controller_sim',
            name='tilt_controller_sim',
            output='log',    
        ),        
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/mpc_status'],
            output='screen'
        )
    ])
