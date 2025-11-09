from setuptools import setup


package_name = 'neodriver_ros'


setup(
name=package_name,
version='0.1.0',
packages=[package_name],
data_files=[
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml', 'README.md']),
('share/' + package_name + '/launch', ['launch/neodriver.launch.py']),
('share/' + package_name + '/srv', ['srv/SetPixel.srv', 'srv/Fill.srv']),
],
install_requires=['setuptools'],
zip_safe=True,
maintainer='Your Name',
maintainer_email='you@example.com',
description='ROS 2 Humble driver for Adafruit NeoDriver (I2C → NeoPixel) on Jetson',
license='MIT',
tests_require=['pytest'],
entry_points={
'console_scripts': [
'neodriver_node = neodriver_ros.neodriver_node:main',
],
},
)
