from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # parameters
    joint_name = DeclareLaunchArgument(
            'joint_name',
            default_value='joint',
            description='joint name'
    )

    led_pin_1 = DeclareLaunchArgument(
            'led_pin_1',
            default_value='5',
            description='defines first led pin'
    )
    led_pin_2 = DeclareLaunchArgument(
            'led_pin_2',
            default_value='6',
            description='defines second led pin'
    )
    led_pin_3 = DeclareLaunchArgument(
            'led_pin_3',
            default_value='12',
            description='defines third led pin'
    )
    time_delay = DeclareLaunchArgument(
            'time_delay',
            default_value='5',
            description='time between led switches'
    )

    # nodes
    timer_node = Node(
        package="led",
        executable="timer_node",
        namespace=LaunchConfiguration('joint_name'),
        name="timer",
        output="screen",
        parameters=[
            {
                "joint_name": LaunchConfiguration('joint_name'),
                "time_delay": LaunchConfiguration('time_delay'),
            }
        ],
    )

    led_node = Node(
        package="led",
        executable="led_node",
        namespace=LaunchConfiguration('joint_name'),
        name="timer",
        output="screen",
        parameters=[
            {
                "joint_name": LaunchConfiguration('joint_name'),
                "led_pin_1": LaunchConfiguration('led_pin_1'),
                "led_pin_1": LaunchConfiguration('led_pin_2'),
                "led_pin_1": LaunchConfiguration('led_pin_3'),
            }
        ],
    )

    return LaunchDescription([
        joint_name,
        led_pin_1,
        led_pin_2,
        led_pin_3,
        time_delay,
        led_node,
        timer_node,
    ])
