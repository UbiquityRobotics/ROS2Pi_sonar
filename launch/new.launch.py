from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Your existing Sonar Node
        Node(
            package='pi_sonar',
            executable='pi_sonar',
            name='pi_sonar',
        ),

        # 2. The Transform Node
        # This attaches the sonar to the robot's base_link.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar_tf',
            # Arguments: x y z yaw pitch roll parent_frame child_frame
            # Example below: Sonar is 10cm forward (x=0.1) from center
            arguments=['0.1', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'sonar_frame']
        ),
    ])

