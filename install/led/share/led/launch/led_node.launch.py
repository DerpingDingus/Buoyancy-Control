from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sensors",
            executable="leak_sensor_node",
            name="leak_sensor",
            output="screen",
            parameters=[
                {
                    "joint_name": "Leak Sensor",
                    "gpio_pin": 22,
                    "pull": "DOWN",

                }
            ],
        )
    ])
