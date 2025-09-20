from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='identifier',
            executable='identified_result_publisher',
            name='armor_identifier_node',
            output='screen'
        )
    ])