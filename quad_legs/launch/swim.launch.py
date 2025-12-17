from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch-time args
    frequency = LaunchConfiguration('frequency_hz')
    offset_deg = LaunchConfiguration('group_offset_deg')
    motor_type = LaunchConfiguration('motor_type')
    can_iface  = LaunchConfiguration('can_interface')

    return LaunchDescription([
        DeclareLaunchArgument('frequency_hz', default_value='0.5'),
        DeclareLaunchArgument('sweep_angle_deg', default_value='30.0'),
        DeclareLaunchArgument('centerline_angle_Q3_deg', default_value='45.0'), # angle from the negative x axis
        DeclareLaunchArgument('motor_type', default_value='AK40-10'),
        DeclareLaunchArgument('can_interface', default_value='can0'),

        # Four motors (IDs 1..4)
        Node(package='quad_legs', executable='motor_node', name='motor1',
             parameters=[{
                 'can_interface': can_iface,
                 'can_id': 1,
                 'motor_type': motor_type,
                 'control_hz': 50.0,
                 'auto_start': True,
                 'reverse_polarity': True,
                 'joint_name': 'motor1',
             }]),
        Node(package='quad_legs', executable='motor_node', name='motor2',
             parameters=[{
                 'can_interface': can_iface,
                 'can_id': 2,
                 'motor_type': motor_type,
                 'control_hz': 50.0,
                 'auto_start': True,
                 'reverse_polarity': False,
                 'joint_name': 'motor2',
             }]),
        Node(package='quad_legs', executable='motor_node', name='motor3',
             parameters=[{
                 'can_interface': can_iface,
                 'can_id': 3,
                 'motor_type': motor_type,
                 'control_hz': 50.0,
                 'auto_start': True,
                 'reverse_polarity': True,
                 'joint_name': 'motor3',
             }]),
        Node(package='quad_legs', executable='motor_node', name='motor4',
             parameters=[{
                 'can_interface': can_iface,
                 'can_id': 4,
                 'motor_type': motor_type,
                 'control_hz': 50.0,
                 'auto_start': True,
                 'reverse_polarity': False,
                 'joint_name': 'motor4',
             }]),

        # Swim Node
        Node(package='quad_legs', executable='swim', name='swim',
             parameters=[{
                 'frequency_hz': frequency,
                 'group_offset_deg': offset_deg,
                 'fast_band_deg': 30.0,
                 'kp': 0.0,
                 'kd': 1.0,
                 'max_vel_abs': 30.0,
                 'control_hz': 100.0,
                 'auto_start': False,  # motor nodes already auto_start
                 'auto_zero': False,
             }]),
    ])

