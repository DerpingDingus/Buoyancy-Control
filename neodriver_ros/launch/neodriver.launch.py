from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument('i2c_bus', default_value='1'),
		DeclareLaunchArgument('i2c_addr', default_value='0x60'),
		DeclareLaunchArgument('num_pixels', default_value='2'),
		DeclareLaunchArgument('brightness', default_value='0.3'),
		DeclareLaunchArgument('pixel_order', default_value='RGB'),
		DeclareLaunchArgument('neopixel_pin', default_value='18'),
		DeclareLaunchArgument('use_topics', default_value='true'),


		Node(
			package='neodriver_ros',
			executable='neodriver_node',
			name='neodriver',
			parameters=[{
				'i2c_bus': LaunchConfiguration('i2c_bus'),
				'i2c_addr': LaunchConfiguration('i2c_addr'),
				'num_pixels': LaunchConfiguration('num_pixels'),
				'brightness': LaunchConfiguration('brightness'),
				'pixel_order': LaunchConfiguration('pixel_order'),
				'neopixel_pin': LaunchConfiguration('neopixel_pin'),
				'use_topics': LaunchConfiguration('use_topics'),
			}]
		)
	])
