from setuptools import setup

package_name = 'selqie_tmux_ui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tmux_session.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ryan',
    maintainer_email='you@example.com',
    description='Tmux-based control surface that launches SELQIE nodes and provides a console.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tmux_ui = selqie_tmux_ui.tmux_launcher:main',
        ],
    },
)
