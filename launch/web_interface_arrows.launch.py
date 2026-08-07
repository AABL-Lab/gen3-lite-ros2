import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Serves the arrow-button web teleop UI (arrows/index.html) and a
    rosbridge websocket so it can publish /joy. Run this alongside
    xbox.launch.py controller:=web, then open http://<this host>:8000
    in a browser.
    """
    web_path_arg = DeclareLaunchArgument(
        'web_path',
        default_value='/home/mavis/ros2_ws/src/gen3-lite-ros2/arrows',
        description='Path for the web server to serve files from'
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        ),
        launch_arguments={
            'address': '0.0.0.0',
            'port': '9090'
        }.items()
    )

    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000', '--bind', '0.0.0.0'],
        cwd=LaunchConfiguration('web_path'),
        output='screen'
    )

    return LaunchDescription([
        web_path_arg,
        rosbridge_launch,
        web_server,
    ])
