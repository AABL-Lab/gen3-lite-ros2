#!/usr/bin/env python3
"""
joint_recorder.py — interactively records joint-position waypoints into a
file readable by joint_playback.py.

Jog the arm to a pose (e.g. with joy_teleop.py, MoveIt, or by hand if the
arm supports backdriving), then hit Enter in this script's terminal to
append the arm's current position to the output file — in exactly the
format joint_playback.py parses, so record-then-replay needs no manual
transcription.

Subscribes:
    /joint_states                          (sensor_msgs/JointState)

Usage:
    ros2 run gen3-lite-ros2 joint_recorder <output_file> [duration]

    <output_file>  path to record waypoints to. If it already exists, new
                    waypoints are appended after its current contents.
    [duration]     seconds each recorded waypoint should take to reach
                    from the previous one; written into every line so
                    joint_playback.py doesn't fall back to its own
                    default. Defaults to DEFAULT_DURATION.

Once running, this does NOT need the robot to be driven by any particular
controller — it only reads /joint_states — so it can run alongside
joy_teleop.py, MoveIt, or any other way of moving the arm.

Commands (typed at the prompt, Enter to submit):
    <blank>   record the arm's current pose as the next waypoint
    u         undo (remove) the last waypoint recorded THIS session
    q         quit

The file is rewritten after every record/undo, so it's always valid to
hand to joint_playback.py, even if you exit without typing 'q'.
"""
import os
import sys
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


# Must match JOINT_NAMES in joint_playback.py — the two files' waypoint
# lines are meant to be interchangeable.
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

DEFAULT_DURATION = 4.0  # seconds

HEADER = [
    "# Waypoints recorded by joint_recorder.py — readable by joint_playback.py.\n",
    "# " + ", ".join(JOINT_NAMES) + ", duration\n",
]


class JointRecorder(Node):
    """Tracks the latest /joint_states and appends requested poses to a file."""

    def __init__(self, output_path, duration):
        super().__init__('joint_recorder')
        self._output_path = output_path
        self._duration = duration
        self._lock = threading.Lock()
        self._positions = None  # dict: joint name -> position, from the latest message
        self._recorded_count = 0  # waypoints recorded this session, for 'undo'

        if os.path.exists(output_path):
            with open(output_path) as f:
                self._lines = f.readlines()
        else:
            self._lines = list(HEADER)
            self._write_file()

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg):
        with self._lock:
            self._positions = dict(zip(msg.name, msg.position))

    def _write_file(self):
        with open(self._output_path, 'w') as f:
            f.writelines(self._lines)

    def current_pose(self):
        """Return the current [joint_1..joint_6] positions, or None if not yet available."""
        with self._lock:
            positions = self._positions
        if positions is None:
            return None
        missing = [j for j in JOINT_NAMES if j not in positions]
        if missing:
            self.get_logger().warn(f"/joint_states is missing joint(s): {missing}")
            return None
        return [positions[j] for j in JOINT_NAMES]

    def record(self):
        """Append the arm's current pose to the output file. Returns the formatted line, or None."""
        pose = self.current_pose()
        if pose is None:
            print("No joint state received yet — is the robot driver running?")
            return None

        line = ", ".join(f"{p:.6f}" for p in pose) + f", {self._duration:.2f}\n"
        self._lines.append(line)
        self._recorded_count += 1
        self._write_file()
        return line

    def undo(self):
        """Remove the most recently recorded waypoint from this session."""
        if self._recorded_count == 0:
            print("Nothing to undo.")
            return
        removed = self._lines.pop()
        self._recorded_count -= 1
        self._write_file()
        print(f"Removed: {removed.strip()}")


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: joint_recorder.py <output_file> [duration]")
        sys.exit(1)
    output_path = sys.argv[1]
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_DURATION

    rclpy.init(args=args)
    node = JointRecorder(output_path, duration)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"Recording joints {JOINT_NAMES} to '{output_path}' (duration={duration:.2f}s/waypoint).")
    print("<Enter> record   u = undo   q = quit")

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == 'q':
                break
            elif cmd == 'u':
                node.undo()
            elif cmd == '':
                line = node.record()
                if line is not None:
                    print(f"Recorded: {line.strip()}")
            else:
                print("Unknown command. <Enter> record, 'u' undo, 'q' quit.")
    except (KeyboardInterrupt, EOFError):
        print()

    print(f"Done. {node._recorded_count} waypoint(s) recorded this session -> '{output_path}'.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
