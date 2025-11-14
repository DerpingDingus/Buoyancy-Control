from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    session_name = LaunchConfiguration('session_name')
    left_command = LaunchConfiguration('launch_command')
    console_command = LaunchConfiguration('console_command')

    return LaunchDescription([
        DeclareLaunchArgument(
            'session_name',
            default_value='selqie_ui',
            description='Name of the tmux session to create or attach to.',
        ),
        DeclareLaunchArgument(
            'launch_command',
            default_value='ros2 launch quad_legs quad_full.launch.py',
            description='Command that starts all control nodes in the left pane.',
        ),
        DeclareLaunchArgument(
            'console_command',
            default_value='',
            description='Optional custom console command for the right pane.',
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'selqie_tmux_ui', 'tmux_ui',
                '--session-name', session_name,
                '--launch-command', left_command,
                '--console-command', console_command,
            ],
            output='screen',
            emulate_tty=True,
        ),
    ])
