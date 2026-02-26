from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="water_sensors",
            executable="leak_sensor_node",
            name="leak_sensor",
            output="screen",
            parameters=[
                {
                    "gpio_pin": 23,
                    "pull": "DOWN",

                }
            ],
        )
    ])
