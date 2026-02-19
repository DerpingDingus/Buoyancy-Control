#!/usr/bin/env python3
"""
Launch file for CubeMars servo motor + Raspberry Pi button control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ── Launch arguments ────────────────────────────────────────────────────────
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

    led_pin = DeclareLaunchArgument(
            'led_pin',
            default_value='26',
            description='Defines the LED pin'
    )


    # ── Nodes ───────────────────────────────────────────────────────────────────

    # Servo motor driver node ── from the OTHER package (likely 'servo')
    servo_node = Node(
        package='servo',                        # ← CHANGE THIS to your actual package name
        executable='servo_motor_node',          # ← CHANGE if the executable has different name
        # name=PythonExpression(["'servo_' + '", LaunchConfiguration('joint_name'), "'"]),  # optional
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

            # add auto_start: True, reverse_polarity: False, etc. if wanted
        }]
    )

    # Button control node ── from THIS package
    button_node = Node(
        package='buoy_control',
        executable='button_control_node',
        # name=PythonExpression(["'button_ctrl_' + '", LaunchConfiguration('joint_name'), "'"]),  # optional
        namespace=LaunchConfiguration('joint_name'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name'),
            'button_pin_cw': LaunchConfiguration('button_pin_cw'),
            'button_pin_ccw': LaunchConfiguration('button_pin_ccw'),
            'led_pin': LaunchConfiguration('led_pin'),
            'debounce_time': LaunchConfiguration('debounce_time'),
        }]
    )

    leak_node = Node(
        package='leak_sensor',                        
        executable='leak_sensor_node',
        name = "leak_sensor",
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gpio_pin': LaunchConfiguration('gpio_pin'),
            'pull': LaunchConfiguration('pull'),
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
        led_pin,
        pull,

        servo_node,
        button_node,
        leak_node,
    ])
