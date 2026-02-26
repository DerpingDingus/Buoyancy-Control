#!/usr/bin/env python3
"""
Launch file for CubeMars servo motor + Raspberry Pi button control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    # -- Launch arguments --------------------------------------------------------
    joint_name = DeclareLaunchArgument(
        'joint_name',
        default_value='joint',
        description='Joint name (used for topic namespaces)'
    )

    can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name (socketcan)'
    )

    can_id = DeclareLaunchArgument(
        'can_id',
        default_value='2',
        description='Motor CAN ID (0-255)'
    )

    motor_type = DeclareLaunchArgument(
        'motor_type',
        default_value='AK40-10',
        description='Motor model (affects torque constant)'
    )

    control_hz = DeclareLaunchArgument(
        'control_hz',
        default_value='50.0',
        description='Control loop rate'
    )

    button_pin_cw = DeclareLaunchArgument(
        'button_pin_cw',
        default_value='3',
        description='GPIO pin (BCM) for +90° button'
    )

    button_pin_ccw = DeclareLaunchArgument(
        'button_pin_ccw',
        default_value='5',
        description='GPIO pin (BCM) for -90° button'
    )

    debounce_time = DeclareLaunchArgument(
        'debounce_time',
        default_value='0.05',
        description='Button debounce time (seconds)'
    )

    auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically power on motors'
    )

    gpio_pin = DeclareLaunchArgument(
        'gpio_pin',
        default_value='23',
        description='Defines the leak sensor pin'
    )

    pull = DeclareLaunchArgument(
        'pull',
        default_value='DOWN',
        description='Sets pin state'
    )

    leak_led_pin = DeclareLaunchArgument(
        'leak_led_pin',
        default_value='26',
        description='Defines the LED pin'
    )

    humidity_led_pin = DeclareLaunchArgument(
        'humidity_led_pin',
        default_value='16',
        description='Defines the LED pin'
    )

    dht_pin = DeclareLaunchArgument(
        'dht_pin',
        default_value='6',
        description='Defines the LED pin'
    )

    name = DeclareLaunchArgument(
        'name',
        default_value='humidity_1',
        description='Defines the LED pin'
    )

    humidity_threshold = DeclareLaunchArgument(
        'humidity_threshold',
        default_value='60.0',
        description='Defines the LED pin'
    )

    # -- Nodes -------------------------------------------------------------------

    servo_node = Node(
        package='servo',
        executable='servo_motor_node',
        namespace=LaunchConfiguration('joint_name'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name'),
            'can_interface': LaunchConfiguration('can_interface'),
            'can_id': LaunchConfiguration('can_id'),
            'motor_type': LaunchConfiguration('motor_type'),
            'control_hz': LaunchConfiguration('control_hz'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    button_node = Node(
        package='buoy_control',
        executable='button_control_node',
        namespace=LaunchConfiguration('joint_name'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name'),
            'button_pin_cw': LaunchConfiguration('button_pin_cw'),
            'button_pin_ccw': LaunchConfiguration('button_pin_ccw'),
            'led_pin': LaunchConfiguration('leak_led_pin'),
            'debounce_time': LaunchConfiguration('debounce_time'),
        }]
    )

    leak_node = Node(
        package='water_sensors',
        executable='leak_sensor_node',
        name="leak_sensor",
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gpio_pin': LaunchConfiguration('gpio_pin'),
            'pull': LaunchConfiguration('pull'),
        }]
    )

    humidity_node = Node(
        package='water_sensors',
        executable='humidity_sensor_node',
        name="humidity_sensor",
        output='screen',
        emulate_tty=True,
        parameters=[{
            'led_pin': LaunchConfiguration('humidity_led_pin'),
            'dht_pin': LaunchConfiguration('dht_pin'),
            'humidity_threshold': LaunchConfiguration('humidity_threshold'),
            'name': LaunchConfiguration('name'),
        }]
    )

    return LaunchDescription([
        joint_name,
        can_interface,
        can_id,
        motor_type,
        control_hz,
        button_pin_cw,
        button_pin_ccw,
        debounce_time,
        auto_start,
        gpio_pin,
        leak_led_pin,
        pull,
        humidity_led_pin,
        name,
        humidity_threshold,
        dht_pin,
        servo_node,
        button_node,
        leak_node,
    ])
