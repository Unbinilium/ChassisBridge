from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='chassis_bridge',
            namespace='chassis_bridge',
            executable='bridge',
            name='bridge'
        )
    ])
