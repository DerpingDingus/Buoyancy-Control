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
        'piston_1',
        default_value='Piston_1',
        description='Joint name for piston 1'
    )

    joint_name_2 = DeclareLaunchArgument(
        'piston_2',
        default_value='Piston_2',
        description='Joint name for piston 2'
    )

    joint_name_3 = DeclareLaunchArgument(
        'electric_tube',
        default_value='Electric_tube',
        description='Joint name for electronics tube'
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
        default_value='20',
        description='Defines start delay time'
    )

    delay_time = DeclareLaunchArgument(
        'delay_time',
        default_value='50',
        description='Defines transition delay time'
    )

    gpio_pin_1 = DeclareLaunchArgument(
        'gpio_pin_1',
        default_value='17',
        description='Defines the leak sensor pin'
    )

    gpio_pin_2 = DeclareLaunchArgument(
        'gpio_pin_2',
        default_value='27',
        description='Defines the leak sensor pin'
    )

    gpio_pin_3 = DeclareLaunchArgument(
        'gpio_pin_3',
        default_value='22',
        description='Defines the leak sensor pin'
    )

    pull = DeclareLaunchArgument(
        'pull',
        default_value='DOWN',
        description='Sets pin state'
    )

    #leak_led_pin = DeclareLaunchArgument(
    #    'leak_led_pin',
    #    default_value='26',
    #    description='Defines the LED pin'
    #)

    # -- Nodes -------------------------------------------------------------------

    servo_node_1 = Node(
        package='servo',
        executable='servo_motor_node',
        namespace=LaunchConfiguration('piston_1'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('piston_1'),
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
        namespace=LaunchConfiguration('piston_2'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('piston_1'),
            'can_interface': LaunchConfiguration('can_interface'),
            'can_id': LaunchConfiguration('can_id_2'),
            'motor_type': LaunchConfiguration('motor_type'),
            'control_hz': LaunchConfiguration('control_hz'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    timer_node_1 = Node(
        package='buoy_control',
        executable='timer_control_node',
        namespace=LaunchConfiguration('piston_1'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name': LaunchConfiguration('piston_1'),
            'start_time': LaunchConfiguration('start_time'),
            'delay_time': LaunchConfiguration('delay_time'),
        }]
    )

    leak_node_1 = Node(
        package='sensors',
        executable='leak_sensor_node',
        namespace=LaunchConfiguration('piston_1'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name':LaunchConfiguration('piston_1'),
            'gpio_pin': LaunchConfiguration('gpio_pin_3'),
            'pull': LaunchConfiguration('pull'),
        }]
    )
    
    leak_node_2 = Node(
        package='sensors',
        executable='leak_sensor_node',
        namespace=LaunchConfiguration('piston_2'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name':LaunchConfiguration('piston_2'),
            'gpio_pin': LaunchConfiguration('gpio_pin_2'),
            'pull': LaunchConfiguration('pull'),
        }]
    )
    
    leak_node_3 = Node(
        package='sensors',
        executable='leak_sensor_node',
        namespace=LaunchConfiguration('electric_tube'),
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_name':LaunchConfiguration('electric_tube'),
            'gpio_pin': LaunchConfiguration('gpio_pin_1'),
            'pull': LaunchConfiguration('pull'),
        }]
    )


    return LaunchDescription([
        joint_name_1,
        joint_name_2,
        joint_name_3,
        can_interface,
        can_id_1,
        can_id_2,
        motor_type,
        control_hz,
        auto_start,
        start_time,
        delay_time,
        gpio_pin_1,
        gpio_pin_2,
        gpio_pin_3,
        pull,
        servo_node_1,
        servo_node_2,
        timer_node_1,
        leak_node_1,
        leak_node_2,
        leak_node_3,
    ])
