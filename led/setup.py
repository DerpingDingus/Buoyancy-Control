from setuptools import setup, find_packages

package_name = 'led'

setup(
    name=package_name,
    version='0.0.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/led_node.launch.py',
            ],
        ),
    ],
    install_requires=[
        'setuptools',
        'spidev',
        'rclpy',
        'std_msgs',
    ],
    zip_safe=True,
    maintainer='DerpingDingus',
    maintainer_email='',
    description='Status LED node for buoyancy device',
    license='MIT',
    entry_points={
        'console_scripts': [
            'led_node = led.led_node:main',
        ],
    },
)

