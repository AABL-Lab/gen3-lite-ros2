from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # --- MoveIt config: gen3_lite instead of gen3 ---
    moveit_config = (
        MoveItConfigsBuilder(
            'gen3_lite',
            package_name='kinova_gen3_lite_moveit_config'
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .to_moveit_configs()
    )

    # --- Bring up the arm ---
    # kortex_bringup's generic launch file takes robot_type/dof/gripper args
    # rather than separate per-arm launch files. Gen3 Lite is 6DOF and has
    # its own built-in 2f gripper (not the Robotiq 2f-85 used on the 7DOF arm).
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('kortex_bringup'),
                'launch',
                'gen3_lite.launch.py'   # dedicated gen3_lite launch file
            )
        ),
        launch_arguments={
            'use_fake_hardware': 'false',
            'robot_ip': '192.168.1.10',
            'gripper': 'gen3_lite_2f',      # built-in gripper, not robotiq_2f_85
            # kortex_control.launch.py spawns two independent controller
            # slots: robot_controller (always ACTIVE) and robot_pos_controller
            # (always spawned --inactive). We want twist_controller active
            # for teleop but joint_trajectory_controller still LOADED
            # (just inactive) so `ros2 control switch_controllers` can
            # activate it later for playback. Without this override,
            # robot_pos_controller keeps its own default ('twist_controller'),
            # so joint_trajectory_controller never gets loaded at all —
            # switch_controllers then fails with "no controller with this
            # name exists".
            'robot_controller': 'twist_controller',
            'robot_pos_controller': 'joint_trajectory_controller',
            'launch_rviz': 'false',
        }.items()
    )

    controller_arg = DeclareLaunchArgument('controller', default_value='xbox')

    # Deactivate joint_trajectory_controller and activate twist_controller
    switch_controller = TimerAction(
        period=5.0,  # wait for ros2_control to finish loading
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'twist_controller',
                    '--activate-as-group',
                    '--deactivate',
                    'joint_trajectory_controller',
                ],
                output='screen',
            )
        ]
    )

    joy_teleop = Node(
        package='gen3-lite-ros2',
        executable='joy_teleop',
        arguments=[LaunchConfiguration('controller')],
        name='joy_teleop',
        output='screen'
    )

    # recorder_node = Node(
    #     package="AZ_demo",
    #     executable="recorder",
    #     name="recorder",
    #     output="screen",
    #     parameters=[{
    #         "topics": [
    #             "/joy",
    #             "/twist_controller/commands",
    #             "/joint_states",
    #         ],
    #         "prefix": LaunchConfiguration('controller'),
    #     }],
    # )

    return LaunchDescription([
        controller_arg,
        robot_launch,
        switch_controller,
        joy_teleop,
        # recorder_node,  # uncomment once the node definition above is uncommented
    ])