from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='mjbots_power_dist',
                executable='power_dist_node',
                name='mjbots_power_dist',
                output='screen',
                parameters=[
                    {
                        'can_interface': 'can0',
                        'status_id': 0x0500,
                        'poll_hz': 50.0,
                    }
                ],
            )
        ]
    )
