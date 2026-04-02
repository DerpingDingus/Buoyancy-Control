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
    joint_name_1 = DeclareLaunchArgument(
        'joint_name_1',
        default_value='joint_1',
        description='Joint name for motor 1'
    )

    joint_name_2 = DeclareLaunchArgument(
        'joint_name_2',
        default_value='joint_2',
        description='Joint name for motor 2'
    )

    can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name (socketcan)'
    )

    can_id_1 = DeclareLaunchArgument(
        'can_id_1',
        default_value='1',
        description='Motor CAN ID (0-255)'
    )

    can_id_2 = DeclareLaunchArgument(
        'can_id_2',
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

    auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Automatically power on motors'
    )

    start_time = DeclareLaunchArgument(
        'start_time',
        default_value='5',
        description='Defines start delay time'
    )

    # -- Nodes -------------------------------------------------------------------

    servo_node_1 = Node(
        package='servo',
        executable='servo_motor_node',
        namespace=LaunchConfiguration('joint_name_1'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name_1'),
            'can_interface': LaunchConfiguration('can_interface'),
            'can_id': LaunchConfiguration('can_id_1'),
            'motor_type': LaunchConfiguration('motor_type'),
            'control_hz': LaunchConfiguration('control_hz'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    servo_node_2 = Node(
        package='servo',
        executable='servo_motor_node',
        namespace=LaunchConfiguration('joint_name_2'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name_2'),
            'can_interface': LaunchConfiguration('can_interface'),
            'can_id': LaunchConfiguration('can_id_2'),
            'motor_type': LaunchConfiguration('motor_type'),
            'control_hz': LaunchConfiguration('control_hz'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    zero_node_1 = Node(
        package='buoy_control',
        executable='zero_motor_node',
        namespace=LaunchConfiguration('joint_name_1'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name_1'),
        }]
    )

    zero_node_2 = Node(
        package='buoy_control',
        executable='zero_motor_node',
        namespace=LaunchConfiguration('joint_name_2'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('joint_name_2'),
        }]
    )


    return LaunchDescription([
        joint_name_1,
        joint_name_2,
        can_interface,
        can_id_1,
        can_id_2,
        motor_type,
        control_hz,
        auto_start,
        servo_node_1,
        servo_node_2,
        zero_node_1,
        zero_node_2
    ])
