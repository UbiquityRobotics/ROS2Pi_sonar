from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pi_sonar',
            executable='pi_sonar',
            name='pi_sonar',
        ),
    ])
