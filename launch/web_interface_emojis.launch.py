from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Serves the emoji action panel web UI (emojis/index.html) and accepts
    its /emoji_action WebSocket connection — both via emoji_web_bridge.py.
    Open http://<this host>:<http_port> in a browser once running.

    Uses different default ports (8001/9091) than web_interface_arrows.launch.py
    (8000/9090) so both web interfaces can run at the same time.

    See joy_web_bridge.py / web_interface_arrows.launch.py for why this
    doesn't use rosbridge_server.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'web_path',
            default_value='/home/mavis/ros2_ws/src/gen3-lite-ros2/emojis',
            description='Path for the web server to serve files from',
        ),
        DeclareLaunchArgument('http_port', default_value='8001'),
        DeclareLaunchArgument('ws_port', default_value='9091'),
        Node(
            package='gen3-lite-ros2',
            executable='emoji_web_bridge',
            arguments=[
                LaunchConfiguration('web_path'),
                LaunchConfiguration('http_port'),
                LaunchConfiguration('ws_port'),
            ],
            name='emoji_web_bridge',
            output='screen',
        ),
    ])
