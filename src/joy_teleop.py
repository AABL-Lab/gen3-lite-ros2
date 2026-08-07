#!/usr/bin/env python3
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



# ─── CONTROLLER MAPPING ──────────────────────────────────────────────────────

# For xbox

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



# ─── SPEED SETTINGS ──────────────────────────────────────────────────────────

LINEAR_SCALE = 0.15   # m/s

ANGULAR_SCALE = 15     # rad/s

TURBO_MULT = 2.5



# ─── PUBLISH RATE ────────────────────────────────────────────────────────────

# Publish at a fixed rate regardless of joy callback rate.

# The kortex twist controller expects a steady heartbeat — going silent

# causes BaseCyclicClient::RefreshFeedback timeouts. 25 Hz is plenty.

PUBLISH_HZ = 25



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

    def __init__(self):

        super().__init__('joy_teleop')



        # ── TF ───────────────────────────────────────────────────────────────

        self._tf_buffer = Buffer()

        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._R_base_eef = np.eye(3)

        self._tf_ready = False

        self._tf_check_timer = self.create_timer(1.0, self._check_tf_ready)



        # ── Publishers / clients ──────────────────────────────────────────────

        self._gripper_client = ActionClient(

            self, ParallelGripperCommand,

            '/robotiq_gripper_controller/gripper_cmd'

        )

        self._twist_pub = self.create_publisher(

            Twist, '/twist_controller/commands', 10

        )

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)



        # ── State ─────────────────────────────────────────────────────────────

        self._last_axes = []

        self._last_buttons = []

        self.gripper_busy = False

        self.translation_mode = True

        self._rb_held = False



        # _current_twist is what we want to send right now.

        # The joy callback updates it; the timer publishes it at a fixed rate.

        # This gives the kortex driver a steady heartbeat so it never times out.

        self._current_twist = Twist()


        self.create_timer(1.0 / PUBLISH_HZ, self._publish_timer_cb)



        self.get_logger().info("=" * 50)

        self.get_logger().info("JoyTeleop ready  [BASE-FRAME mode]")

        self.get_logger().info(f"  Base frame : {BASE_FRAME}")

        self.get_logger().info(f"  EEF frame  : {EEF_FRAME}")

        self.get_logger().info("  Hold RB          : enable arm movement")

        self.get_logger().info("  Y button         : toggle translate/rotate mode")

        self.get_logger().info("  TRANSLATE mode   : left stick = X/Y, right stick V = Z")

        self.get_logger().info("  ROTATE mode      : left stick = roll/pitch, right stick H = yaw")

        self.get_logger().info("  LT               : open gripper")

        self.get_logger().info("  RT               : close gripper")

        self.get_logger().info("=" * 50)



    # ── Fixed-rate publish ────────────────────────────────────────────────────

    def _publish_timer_cb(self):

        """

        Publish _current_twist at PUBLISH_HZ.

        Sends zeros when RB is not held — the kortex driver needs this

        steady stream to keep BaseCyclicClient::RefreshFeedback alive.

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

            idx = AXES[name]

            return self._last_axes[idx] if idx < len(self._last_axes) else 1.0



        # ── Mode toggle ───────────────────────────────────────────────────────

        if btn('Y') == 1 and last_btn('Y') == 0:

            self.translation_mode = not self.translation_mode

            self.get_logger().info(

                "TRANSLATION mode" if self.translation_mode else "ROTATION mode"

            )



        # ── Gripper ───────────────────────────────────────────────────────────

        lt = axis('LT')

        rt = axis('RT')



        if lt < -TRIGGER_THRESHOLD and last_axis('LT') >= -TRIGGER_THRESHOLD:

            if not self.gripper_busy:

                self.get_logger().info("LT -> Opening gripper")

                self.send_gripper(GRIPPER_OPEN)



        if rt < -TRIGGER_THRESHOLD and last_axis('RT') >= -TRIGGER_THRESHOLD:

            if not self.gripper_busy:

                self.get_logger().info("RT -> Closing gripper")

                self.send_gripper(GRIPPER_CLOSE)



        # ── Arm movement ──────────────────────────────────────────────────────

        # Update _current_twist based on joystick state.

        # The timer publishes it at a fixed rate for a steady kortex heartbeat.

        rb = btn('RB')



        if rb:

            self._update_eef_rotation()



            speed = TURBO_MULT if btn('LB') else 1.0

            lin = LINEAR_SCALE  * speed

            ang = ANGULAR_SCALE * speed



            twist = Twist()



            if self.translation_mode:

                v_base = np.array([

                    axis('left_stick_v')  * lin,   # +X forward in base

                    axis('left_stick_h') * lin,   # +Y left in base

                    axis('right_stick_v')  * lin,   # +Z up in base

                ])

                v_eef, _ = rotate_twist_to_base(v_base, np.zeros(3), self._R_base_eef)

                twist.linear.x = float(v_eef[0])

                twist.linear.y = float(v_eef[1])

                twist.linear.z = float(v_eef[2])

            else:

                w_base = np.array([

                    axis('right_stick_v')  * ang,   # roll  around base X

                    -axis('left_stick_v')  * ang,   # pitch around base Y

                    -axis('left_stick_h') * ang,   # yaw   around base Z

                ])

                _, w_eef = rotate_twist_to_base(np.zeros(3), w_base, self._R_base_eef)

                twist.angular.x = float(w_eef[0])

                twist.angular.y = float(w_eef[1])

                twist.angular.z = float(w_eef[2])



            self._current_twist = twist



        else:

            if self._rb_held:

                self.get_logger().info("RB released — arm stopped.")

            self._current_twist = Twist()   # zeros — arm holds still, heartbeat continues



        self._rb_held = bool(rb)

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