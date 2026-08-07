from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- Bring up the arm ---
    # kortex_bringup's gen3_lite launch file defaults to
    # robot_controller=joint_trajectory_controller, which is what
    # joint_playback needs — no controller switch required (contrast with
    # xbox.launch.py, which switches over to twist_controller for teleop).
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kortex_bringup'),
                'launch',
                'gen3_lite.launch.py'
            )
        ),
        launch_arguments={
            'use_fake_hardware': 'false',
            'robot_ip': LaunchConfiguration('robot_ip'),
            'gripper': 'gen3_lite_2f',
            'robot_controller': 'joint_trajectory_controller',
            'launch_rviz': 'false',
        }.items()
    )

    joint_playback = Node(
        package='gen3-lite-ros2',
        executable='joint_playback',
        arguments=[LaunchConfiguration('waypoints_file')],
        name='joint_playback',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoints_file',
            description='Path to the waypoint file to play back (see joint_playback.py for the file format).',
        ),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.10'),
        robot_launch,
        joint_playback,
    ])
