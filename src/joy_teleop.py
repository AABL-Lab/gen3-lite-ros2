#!/usr/bin/env python3
"""
joy_teleop.py — gamepad teleoperation for the Kinova Gen3 Lite arm.

Reads a sensor_msgs/Joy stream and drives the arm's Cartesian twist
controller plus the parallel gripper. Stick input is interpreted in the
BASE frame, then rotated into the EEF frame (via TF) before publishing,
so "push stick forward" always moves the arm away from the robot's base
regardless of the current wrist orientation.

Subscribes:
    /joy                                    (sensor_msgs/Joy)

Publishes:
    /twist_controller/commands              (geometry_msgs/Twist)
    at a fixed PUBLISH_HZ, so the kortex driver gets a steady heartbeat
    even while no buttons are held (it sends zero twists otherwise).

Action client:
    /robotiq_gripper_controller/gripper_cmd (control_msgs/action/ParallelGripperCommand)

Usage:
    ros2 run <pkg> joy_teleop.py [xbox|web]

    xbox  - also launches `ros2 run joy joy_node` as a subprocess, so a
            wired/Bluetooth Xbox controller publishes /joy directly.
    web   - (default) assumes something else already publishes /joy,
            e.g. a browser Gamepad API bridge. Standard browser gamepads
            report axes/buttons in the same order as joy_node's xbox
            mapping, so no separate mapping is needed.

Default controls (see "CUSTOMIZING THE MAPPING" below to change these):
    Hold RB          : enable arm movement (released -> arm holds still)
    Hold LB + RB      : turbo speed
    Y button         : toggle translate / rotate mode
    TRANSLATE mode   : left stick = X/Y, right stick vertical = Z
    ROTATE mode      : left stick = roll/pitch, right stick horizontal = yaw
    LT               : open gripper
    RT               : close gripper

CUSTOMIZING THE MAPPING
------------------------
There are three independent layers, each editable without touching any
other code:

1. AXES / BUTTONS — physical layout of the gamepad. Maps a human-readable
   control name (e.g. 'left_stick_v', 'RB') to the index joy_node reports
   it at. Edit this if you're using a controller whose driver reports
   axes/buttons in a different order.

2. ACTIONS — which physical button/trigger drives each on/off command
   ('enable_move', 'turbo', 'toggle_mode', 'gripper_open', 'gripper_close').
   Change a value here to move a command to a different button, e.g.
   swap 'enable_move': 'RB' for 'enable_move': 'LB' to use the other
   shoulder button. Must reference a key that exists in AXES (for trigger
   actions) or BUTTONS (for button actions).

3. AXIS_MAP — which stick axis drives each Cartesian degree of freedom
   ('translate_x', 'translate_y', 'translate_z', 'rotate_roll',
   'rotate_pitch', 'rotate_yaw'), plus a +1/-1 sign to flip its direction.
   Change the axis name to move a DOF to a different stick, or flip the
   sign to reverse it.

Speed and behavior constants (LINEAR_SCALE, ANGULAR_SCALE, TURBO_MULT,
PUBLISH_HZ, TRIGGER_THRESHOLD, GRIPPER_*) live in their own section below
and can be tuned independently of the mapping.
"""
import subprocess

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from control_msgs.action import ParallelGripperCommand

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

import sys


# ─── FRAME SETTINGS ──────────────────────────────────────────────────────────
BASE_FRAME = 'base_link'
EEF_FRAME = 'end_effector_link'
# ─────────────────────────────────────────────────────────────────────────────


# ─── PHYSICAL CONTROLLER LAYOUT ──────────────────────────────────────────────
# Index each named axis/button is reported at by joy_node. Standard for an
# Xbox controller (and browser Gamepad API gamepads, which use the same
# order). Edit this if a different controller reports indices differently.

AXES = {
    'left_stick_h': 0,
    'left_stick_v': 1,
    'right_stick_h': 3,
    'right_stick_v': 4,
    'LT': 2,
    'RT': 5,
}

BUTTONS = {
    'A': 0,
    'B': 1,
    'X': 2,
    'Y': 3,
    'LB': 4,
    'RB': 5,
    'back': 6,
    'start': 7,
}
# ─────────────────────────────────────────────────────────────────────────────


# ─── COMMAND MAPPING ─────────────────────────────────────────────────────────
# Which physical control (from AXES/BUTTONS above) drives each command.
# Remap a command by changing its value here — no other code needs to change.

ACTIONS = {
    'enable_move':   'RB',   # hold to allow arm movement
    'turbo':         'LB',   # hold with enable_move for TURBO_MULT speed
    'toggle_mode':   'Y',    # tap to switch translate/rotate mode
    'gripper_open':  'LT',   # trigger axis, pulled past TRIGGER_THRESHOLD
    'gripper_close': 'RT',   # trigger axis, pulled past TRIGGER_THRESHOLD
}

# Which stick axis drives each Cartesian degree of freedom, and its sign.
# Flip the sign to reverse a direction; change the axis name to move a DOF
# to a different stick. Only used while ACTIONS['enable_move'] is held.

AXIS_MAP = {
    'translate_x':  ('left_stick_v',  +1),
    'translate_y':  ('left_stick_h',  +1),
    'translate_z':  ('right_stick_v', +1),
    'rotate_roll':  ('right_stick_v', +1),
    'rotate_pitch': ('left_stick_v',  -1),
    'rotate_yaw':   ('left_stick_h',  -1),
}
# ─────────────────────────────────────────────────────────────────────────────


# ─── SPEED SETTINGS ──────────────────────────────────────────────────────────
LINEAR_SCALE = 0.15   # m/s
ANGULAR_SCALE = 15    # rad/s
TURBO_MULT = 2.5
# ─────────────────────────────────────────────────────────────────────────────


# ─── PUBLISH RATE ────────────────────────────────────────────────────────────
# Publish at a fixed rate regardless of joy callback rate.
# The kortex twist controller expects a steady heartbeat — going silent
# causes BaseCyclicClient::RefreshFeedback timeouts. 25 Hz is plenty.
PUBLISH_HZ = 25
# ─────────────────────────────────────────────────────────────────────────────


# ─── GRIPPER ─────────────────────────────────────────────────────────────────
TRIGGER_THRESHOLD = 0.5
GRIPPER_OPEN = 0.0
GRIPPER_CLOSE = 0.8
GRIPPER_MAX_EFFORT = 10.0
GRIPPER_JOINT = 'robotiq_85_left_knuckle_joint'
# ─────────────────────────────────────────────────────────────────────────────


def quat_to_rotation_matrix(qx, qy, qz, qw):
    """Convert a quaternion to a 3x3 rotation matrix (base <- eef)."""
    R = np.array([
        [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
    ])
    return R


def rotate_twist_to_base(v_base, w_base, R_base_eef):
    """
    Convert a desired twist in BASE frame into EEF frame for the twist controller.
    v_eef = R^T * v_base  (R maps EEF->base, so R^T maps base->EEF)
    """
    return R_base_eef.T @ v_base, R_base_eef.T @ w_base


class JoyTeleop(Node):
    """Translates /joy input into arm twist commands and gripper goals.

    See the module docstring for the default control scheme and how to
    customize the ACTIONS / AXIS_MAP mapping.
    """

    def __init__(self):
        super().__init__('joy_teleop')

        # ── TF ───────────────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._R_base_eef = np.eye(3)
        self._tf_ready = False
        self._tf_check_timer = self.create_timer(1.0, self._check_tf_ready)

        # ── Publishers / clients ────────────────────────────────────────────
        self._gripper_client = ActionClient(
            self, ParallelGripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )
        self._twist_pub = self.create_publisher(
            Twist, '/twist_controller/commands', 10
        )
        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # ── State ────────────────────────────────────────────────────────────
        self._last_axes = []
        self._last_buttons = []
        self.gripper_busy = False
        self.translation_mode = True
        self._enable_held = False

        # _current_twist is what we want to send right now.
        # The joy callback updates it; the timer publishes it at a fixed rate.
        # This gives the kortex driver a steady heartbeat so it never times out.
        self._current_twist = Twist()

        self.create_timer(1.0 / PUBLISH_HZ, self._publish_timer_cb)

        tx_axis = AXIS_MAP['translate_x'][0]
        ty_axis = AXIS_MAP['translate_y'][0]
        tz_axis = AXIS_MAP['translate_z'][0]
        rr_axis = AXIS_MAP['rotate_roll'][0]
        rp_axis = AXIS_MAP['rotate_pitch'][0]
        ry_axis = AXIS_MAP['rotate_yaw'][0]

        self.get_logger().info("=" * 50)
        self.get_logger().info("JoyTeleop ready  [BASE-FRAME mode]")
        self.get_logger().info(f"  Base frame : {BASE_FRAME}")
        self.get_logger().info(f"  EEF frame  : {EEF_FRAME}")
        self.get_logger().info(f"  Hold {ACTIONS['enable_move']:<4}       : enable arm movement")
        self.get_logger().info(f"  {ACTIONS['toggle_mode']:<4} button   : toggle translate/rotate mode")
        self.get_logger().info(f"  TRANSLATE mode   : {tx_axis}=X  {ty_axis}=Y  {tz_axis}=Z")
        self.get_logger().info(f"  ROTATE mode      : {rr_axis}=roll  {rp_axis}=pitch  {ry_axis}=yaw")
        self.get_logger().info(f"  {ACTIONS['gripper_open']:<4}             : open gripper")
        self.get_logger().info(f"  {ACTIONS['gripper_close']:<4}             : close gripper")
        self.get_logger().info("=" * 50)

    # ── Fixed-rate publish ────────────────────────────────────────────────────
    def _publish_timer_cb(self):
        """
        Publish _current_twist at PUBLISH_HZ.
        Sends zeros when arm movement is not enabled — the kortex driver needs
        this steady stream to keep BaseCyclicClient::RefreshFeedback alive.
        """
        self._twist_pub.publish(self._current_twist)

    # ── TF startup check ─────────────────────────────────────────────────────
    def _check_tf_ready(self):
        try:
            self._tf_buffer.lookup_transform(
                BASE_FRAME, EEF_FRAME, rclpy.time.Time()
            )
            self._tf_ready = True
            self._tf_check_timer.cancel()
            self.get_logger().info(f"TF ready: '{BASE_FRAME}' <- '{EEF_FRAME}'")
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(
                f"Waiting for TF '{BASE_FRAME}' <- '{EEF_FRAME}'...",
                throttle_duration_sec=3.0,
            )

    # ── TF lookup ─────────────────────────────────────────────────────────────
    def _update_eef_rotation(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                BASE_FRAME, EEF_FRAME, rclpy.time.Time()
            )
            q = tf.transform.rotation
            self._R_base_eef = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
            return True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed: {e} — using last known rotation.",
                throttle_duration_sec=2.0,
            )
            return False

    # ── Joy callback ──────────────────────────────────────────────────────────
    def joy_cb(self, msg):
        axes    = list(msg.axes)
        buttons = list(msg.buttons)

        def axis(name):
            idx = AXES[name]
            return axes[idx] if idx < len(axes) else 0.0

        def btn(name):
            idx = BUTTONS[name]
            return buttons[idx] if idx < len(buttons) else 0

        def last_btn(name):
            idx = BUTTONS[name]
            return self._last_buttons[idx] if idx < len(self._last_buttons) else 0

        def last_axis(name):
            # Trigger axes rest at 1.0 (unpressed), not 0.0.
            idx = AXES[name]
            return self._last_axes[idx] if idx < len(self._last_axes) else 1.0

        def action_btn(action):
            """Current state of a logical button command (see ACTIONS)."""
            return btn(ACTIONS[action])

        def action_last_btn(action):
            return last_btn(ACTIONS[action])

        def action_axis(action):
            """Current value of a logical trigger command (see ACTIONS)."""
            return axis(ACTIONS[action])

        def action_last_axis(action):
            return last_axis(ACTIONS[action])

        def mapped_axis(dof):
            """Current value of a Cartesian DOF, per AXIS_MAP, sign-corrected."""
            name, sign = AXIS_MAP[dof]
            return axis(name) * sign

        # ── Mode toggle ───────────────────────────────────────────────────────
        if action_btn('toggle_mode') == 1 and action_last_btn('toggle_mode') == 0:
            self.translation_mode = not self.translation_mode
            self.get_logger().info(
                "TRANSLATION mode" if self.translation_mode else "ROTATION mode"
            )

        # ── Gripper ───────────────────────────────────────────────────────────
        if (action_axis('gripper_open') < -TRIGGER_THRESHOLD
                and action_last_axis('gripper_open') >= -TRIGGER_THRESHOLD):
            if not self.gripper_busy:
                self.get_logger().info("Opening gripper")
                self.send_gripper(GRIPPER_OPEN)

        if (action_axis('gripper_close') < -TRIGGER_THRESHOLD
                and action_last_axis('gripper_close') >= -TRIGGER_THRESHOLD):
            if not self.gripper_busy:
                self.get_logger().info("Closing gripper")
                self.send_gripper(GRIPPER_CLOSE)

        # ── Arm movement ─────────────────────────────────────────────────────
        # Update _current_twist based on joystick state.
        # The timer publishes it at a fixed rate for a steady kortex heartbeat.
        enable = action_btn('enable_move')

        if enable:
            self._update_eef_rotation()

            speed = TURBO_MULT if action_btn('turbo') else 1.0
            lin = LINEAR_SCALE  * speed
            ang = ANGULAR_SCALE * speed

            twist = Twist()

            if self.translation_mode:
                v_base = np.array([
                    mapped_axis('translate_x') * lin,
                    mapped_axis('translate_y') * lin,
                    mapped_axis('translate_z') * lin,
                ])
                v_eef, _ = rotate_twist_to_base(v_base, np.zeros(3), self._R_base_eef)
                twist.linear.x = float(v_eef[0])
                twist.linear.y = float(v_eef[1])
                twist.linear.z = float(v_eef[2])
            else:
                w_base = np.array([
                    mapped_axis('rotate_roll')  * ang,
                    mapped_axis('rotate_pitch') * ang,
                    mapped_axis('rotate_yaw')   * ang,
                ])
                _, w_eef = rotate_twist_to_base(np.zeros(3), w_base, self._R_base_eef)
                twist.angular.x = float(w_eef[0])
                twist.angular.y = float(w_eef[1])
                twist.angular.z = float(w_eef[2])

            self._current_twist = twist
        else:
            if self._enable_held:
                self.get_logger().info(f"{ACTIONS['enable_move']} released — arm stopped.")
            self._current_twist = Twist()   # zeros — arm holds still, heartbeat continues

        self._enable_held = bool(enable)
        self._last_axes = axes
        self._last_buttons = buttons

    # ── Gripper helpers ───────────────────────────────────────────────────────
    def send_gripper(self, position: float):
        if not self._gripper_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Gripper action server not available")
            return

        self.gripper_busy = True

        goal = ParallelGripperCommand.Goal()
        goal.command.name     = [GRIPPER_JOINT]
        goal.command.position = [position]
        goal.command.effort   = [GRIPPER_MAX_EFFORT]

        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            self.gripper_busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Gripper done — reached_goal: {result.reached_goal}, "
            f"stalled: {result.stalled}, "
            f"position: {list(result.state.position)}"
        )
        self.gripper_busy = False


def launch_subprocess(cmd):
    return subprocess.Popen(cmd, shell=True)


def main(args=None):
    controller = sys.argv[1] if len(sys.argv) > 1 else 'web'

    print("Launching joy_node...")
    if controller == 'xbox':
        joy_proc = launch_subprocess("ros2 run joy joy_node")

    rclpy.init(args=args)
    node = JoyTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if controller == 'xbox':
            joy_proc.terminate()
            joy_proc.wait()
        print("All processes stopped.")


if __name__ == '__main__':
    main()
