#!/usr/bin/env python3
"""
behavior_recorder.py — records a "behavior" (a sequence of joint-position
waypoints) using Xbox controller buttons, so you can record without ever
letting go of the sticks. Replay it later with joint_playback.py.

Run this ALONGSIDE joy_teleop.py (e.g. via
`ros2 launch gen3-lite-ros2 xbox.launch.py controller:=xbox`), which is
what actually drives the arm (RB/LB/Y/LT/RT). This script only watches
/joy for its own buttons (see RECORD_ACTIONS — chosen to not overlap with
joy_teleop.py's) and /joint_states for the current pose, so it never
interferes with driving the arm — jog to a pose with joy_teleop.py as
normal, then tap a button here to capture it.

Subscribes:
    /joy                                    (sensor_msgs/Joy)
    /joint_states                           (sensor_msgs/JointState)

Usage:
    ros2 run gen3-lite-ros2 behavior_recorder <output_file> [duration]

    <output_file>  path to record waypoints to. If it already exists, new
                    waypoints are appended after its current contents.
    [duration]     seconds each recorded waypoint should take to reach
                    from the previous one; written into every line so
                    joint_playback.py doesn't fall back to its own
                    default. Defaults to DEFAULT_DURATION.

Controls (Xbox buttons — edit RECORD_ACTIONS below to remap):
    A       record the arm's current pose as the next waypoint
    B       undo (remove) the last waypoint recorded THIS session
    start   stop recording (same as Ctrl+C)

Output is written in exactly the format joint_playback.py parses, and the
file is rewritten after every record/undo, so it's always safe to hand to
joint_playback.py even if you stop with Ctrl+C instead of 'start'.
"""
import os
import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState


# Must match JOINT_NAMES in joint_playback.py — the two files' waypoint
# lines are meant to be interchangeable.
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

DEFAULT_DURATION = 4.0  # seconds

# Physical button layout — must match joy_teleop.py's BUTTONS (same
# controller, same joy_node, so the indices have to agree).
BUTTONS = {
    'A': 0, 'B': 1, 'X': 2, 'Y': 3,
    'LB': 4, 'RB': 5, 'back': 6, 'start': 7,
}

# Which button drives each recording command. Chosen to avoid
# joy_teleop.py's ACTIONS (RB/LB/Y/LT/RT are all taken by arm movement).
RECORD_ACTIONS = {
    'record': 'A',
    'undo':   'B',
    'quit':   'start',
}

HEADER = [
    "# Waypoints recorded by behavior_recorder.py — readable by joint_playback.py.\n",
    "# " + ", ".join(JOINT_NAMES) + ", duration\n",
]


class BehaviorRecorder(Node):
    """Watches /joy for record/undo/quit button edges and /joint_states for the current pose."""

    def __init__(self, output_path, duration):
        super().__init__('behavior_recorder')
        self._output_path = output_path
        self._duration = duration
        self._positions = None    # dict: joint name -> position, latest /joint_states
        self._last_buttons = []
        self._recorded_count = 0  # waypoints recorded this session, for 'undo'
        self.done = False

        if os.path.exists(output_path):
            with open(output_path) as f:
                self._lines = f.readlines()
        else:
            self._lines = list(HEADER)
            self._write_file()

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)

        self.get_logger().info("=" * 50)
        self.get_logger().info("BehaviorRecorder ready")
        self.get_logger().info(f"  Recording to : {output_path}")
        self.get_logger().info(f"  {RECORD_ACTIONS['record']:<5}         : record current pose as next waypoint")
        self.get_logger().info(f"  {RECORD_ACTIONS['undo']:<5}         : undo last waypoint recorded this session")
        self.get_logger().info(f"  {RECORD_ACTIONS['quit']:<5}     : stop recording")
        self.get_logger().info("  (drive the arm with joy_teleop.py running alongside this node)")
        self.get_logger().info("=" * 50)

    def _joint_state_cb(self, msg):
        self._positions = dict(zip(msg.name, msg.position))

    def _write_file(self):
        with open(self._output_path, 'w') as f:
            f.writelines(self._lines)

    def _current_pose(self):
        positions = self._positions
        if positions is None:
            return None
        missing = [j for j in JOINT_NAMES if j not in positions]
        if missing:
            self.get_logger().warn(f"/joint_states is missing joint(s): {missing}")
            return None
        return [positions[j] for j in JOINT_NAMES]

    def _record(self):
        pose = self._current_pose()
        if pose is None:
            self.get_logger().warn("No joint state received yet — is the robot driver running?")
            return
        line = ", ".join(f"{p:.6f}" for p in pose) + f", {self._duration:.2f}\n"
        self._lines.append(line)
        self._recorded_count += 1
        self._write_file()
        self.get_logger().info(f"Recorded waypoint {self._recorded_count}: {line.strip()}")

    def _undo(self):
        if self._recorded_count == 0:
            self.get_logger().info("Nothing to undo.")
            return
        removed = self._lines.pop()
        self._recorded_count -= 1
        self._write_file()
        self.get_logger().info(f"Removed: {removed.strip()}")

    def _joy_cb(self, msg):
        buttons = list(msg.buttons)

        def btn(name):
            idx = BUTTONS[name]
            return buttons[idx] if idx < len(buttons) else 0

        def last_btn(name):
            idx = BUTTONS[name]
            return self._last_buttons[idx] if idx < len(self._last_buttons) else 0

        def pressed(action):
            name = RECORD_ACTIONS[action]
            return btn(name) == 1 and last_btn(name) == 0

        if pressed('record'):
            self._record()
        if pressed('undo'):
            self._undo()
        if pressed('quit'):
            self.get_logger().info("Stop button pressed — ending recording.")
            self.done = True

        self._last_buttons = buttons


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: behavior_recorder.py <output_file> [duration]")
        sys.exit(1)
    output_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_DURATION

    rclpy.init(args=args)
    node = BehaviorRecorder(output_path, duration)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"Done. {node._recorded_count} waypoint(s) recorded this session -> '{output_path}'."
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
