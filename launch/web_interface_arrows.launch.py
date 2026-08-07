from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Serves the arrow-button web teleop UI (arrows/index.html) and accepts
    its /joy WebSocket connection — both via joy_web_bridge.py. Run this
    alongside xbox.launch.py controller:=web, then open
    http://<this host>:<http_port> in a browser.

    This does NOT use rosbridge_server: rosbridge_library's build in this
    workspace currently fails to import (rosidl_pycommon.interface_base_classes
    is missing from the installed rosidl_pycommon — a real version mismatch,
    not fixed by upgrading rosidl_pycommon). joy_web_bridge.py talks to the
    browser directly instead, so this launch file works regardless.
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'web_path',
            default_value='/home/mavis/ros2_ws/src/gen3-lite-ros2/arrows',
            description='Path for the web server to serve files from',
        ),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('ws_port', default_value='9090'),
        Node(
            package='gen3-lite-ros2',
            executable='joy_web_bridge',
            arguments=[
                LaunchConfiguration('web_path'),
                LaunchConfiguration('http_port'),
                LaunchConfiguration('ws_port'),
            ],
            name='joy_web_bridge',
            output='screen',
        ),
    ])
