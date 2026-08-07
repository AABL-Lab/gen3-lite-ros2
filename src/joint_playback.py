#!/usr/bin/env python3
"""
joint_playback.py — plays back a sequence of joint positions on the
Kinova Gen3 Lite arm through joint_trajectory_controller.

Reads a waypoint file (see WAYPOINT FILE FORMAT below) and sends each
waypoint to the controller as its own FollowJointTrajectory goal, one at a
time, waiting for each to finish before sending the next.

Action client:
    /joint_trajectory_controller/follow_joint_trajectory
    (control_msgs/action/FollowJointTrajectory)

This requires joint_trajectory_controller to be the active controller —
it's kortex_bringup's default, but note that xbox.launch.py switches the
robot over to twist_controller for teleop. If the arm was last brought up
for teleop, reactivate joint_trajectory_controller first, e.g.:

    ros2 control switch_controllers \\
        --activate joint_trajectory_controller --deactivate twist_controller

Usage:
    ros2 run gen3-lite-ros2 joint_playback <waypoints_file>
    ros2 launch gen3-lite-ros2 joint_playback.launch.py waypoints_file:=<path>

WAYPOINT FILE FORMAT
---------------------
One waypoint per line, whitespace- or comma-separated:

    joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 [duration]

- The six joint angles are in radians, in JOINT_NAMES order.
- The optional trailing `duration` is how many seconds the controller
  should take to reach that waypoint from the previous one. If omitted,
  DEFAULT_TIME_PER_WAYPOINT is used.
- Blank lines are skipped. `#` starts a comment (to end of line).

Example:
    # home
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    # reach forward, take 2s
    0.3, -0.4, 0.9, 0.0, 1.1, 0.0, 2.0
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# Joint order expected in the waypoint file — must match
# joint_trajectory_controller's `joints` list (see ros2_controllers.yaml).
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

# Time to reach a waypoint when its line doesn't specify its own duration.
DEFAULT_TIME_PER_WAYPOINT = 4.0  # seconds

ACTION_SERVER = '/joint_trajectory_controller/follow_joint_trajectory'


def parse_waypoints(path):
    """Parse a waypoint file into a list of (positions, duration) tuples."""
    n = len(JOINT_NAMES)
    waypoints = []

    with open(path) as f:
        for lineno, raw in enumerate(f, start=1):
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue

            values = [float(v) for v in line.replace(',', ' ').split()]

            if len(values) == n:
                positions, duration = values, DEFAULT_TIME_PER_WAYPOINT
            elif len(values) == n + 1:
                positions, duration = values[:n], values[n]
            else:
                raise ValueError(
                    f"{path}:{lineno}: expected {n} or {n + 1} values, "
                    f"got {len(values)}: {raw.strip()!r}"
                )

            waypoints.append((positions, duration))

    return waypoints


class JointTrajectoryPlayer(Node):
    """Sends a sequence of joint-position waypoints to joint_trajectory_controller, one at a time."""

    def __init__(self, waypoints):
        super().__init__('joint_playback')
        self._waypoints = waypoints
        self._client = ActionClient(self, FollowJointTrajectory, ACTION_SERVER)

    def run(self):
        """Play back all waypoints in order. Returns True iff every one succeeded."""
        self.get_logger().info(f"Waiting for action server '{ACTION_SERVER}'...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f"Action server '{ACTION_SERVER}' not available — "
                "is joint_trajectory_controller active?"
            )
            return False

        total = len(self._waypoints)
        self.get_logger().info(f"Playing back {total} waypoint(s)...")

        for i, (positions, duration) in enumerate(self._waypoints, start=1):
            self.get_logger().info(f"[{i}/{total}] -> {positions} ({duration:.1f}s)")
            if not self._send_and_wait(positions, duration):
                self.get_logger().error(f"Waypoint {i} failed — stopping playback.")
                return False

        self.get_logger().info("Playback complete.")
        return True

    def _send_and_wait(self, positions, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=duration).to_msg()
        goal.trajectory.points = [point]

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(
                f"Goal did not succeed — error_code: {result.error_code}, "
                f"error_string: {result.error_string!r}"
            )
            return False

        return True


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: joint_playback.py <waypoints_file>")
        sys.exit(1)
    waypoints_path = sys.argv[1]

    try:
        waypoints = parse_waypoints(waypoints_path)
    except (OSError, ValueError) as e:
        print(f"Failed to read waypoints file: {e}")
        sys.exit(1)

    if not waypoints:
        print(f"No waypoints found in {waypoints_path}")
        sys.exit(1)

    rclpy.init(args=args)
    node = JointTrajectoryPlayer(waypoints)

    try:
        ok = node.run()
    except KeyboardInterrupt:
        ok = False
        print("\nInterrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
