from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'buoy_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),   # or glob('launch/*launch.[py]*')
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selqie',
    maintainer_email='selqie@todo.todo',
    description='ROS 2 node that controls motors using buttons',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'button_control_node = buoy_control.button_control:main'
        ],
    },
)
