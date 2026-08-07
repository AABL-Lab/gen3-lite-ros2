#!/usr/bin/env python3
"""
joy_web_bridge.py — serves the browser teleop page (arrows/index.html)
and forwards its button/stick state straight to /joy, without going
through rosbridge_server.

Why this exists: rosbridge_server, as currently built in this workspace,
fails to start — rosbridge_library's type_support.py imports
`rosidl_pycommon.interface_base_classes`, which the installed
`rosidl_pycommon` doesn't provide (a real version mismatch between the
pinned rosbridge_suite checkout and this ROS distro's rosidl_pycommon,
confirmed not fixed by upgrading rosidl_pycommon). Rather than touch the
pinned version of rosbridge_suite or rosidl_pycommon, this talks to the
browser directly with a small self-contained WebSocket server (tornado,
already installed) — no rosbridge/rosapi involved, and no external CDN
dependency in the page either.

Serves:
    http://<host>:<http_port>/     static files from <web_path>
    ws://<host>:<ws_port>/joy      the page sends one JSON object per
                                    message: {"axes": [...], "buttons": [...]}
                                    (6 axes, 8 buttons — see joy_teleop.py's
                                    AXES/BUTTONS). Each is published
                                    verbatim to /joy, no server-side state.

Publishes:
    /joy                            (sensor_msgs/Joy)

Usage:
    ros2 run gen3-lite-ros2 joy_web_bridge [web_path] [http_port] [ws_port]

    web_path   directory to serve static files from
               (default: this package's arrows/ directory)
    http_port  default 8000
    ws_port    default 9090 (kept as rosbridge's old default so existing
               docs/URLs and firewall rules don't need to change)
"""
import json
import sys
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

from ament_index_python.packages import get_package_share_directory

import tornado.ioloop
import tornado.web
import tornado.websocket


def _default_web_path():
    # Falls back to the installed copy of arrows/ under this package's
    # share directory. launch/web_interface_arrows.launch.py instead
    # defaults to the source tree directly, so edits to index.html don't
    # need a rebuild when launched that way.
    return get_package_share_directory('gen3-lite-ros2') + '/arrows'


DEFAULT_HTTP_PORT = 8000
DEFAULT_WS_PORT = 9090

AXES_LEN = 6
BUTTONS_LEN = 8


class JoyWebBridge(Node):
    """Publishes /joy from whatever JSON the browser's WebSocket sends."""

    def __init__(self):
        super().__init__('joy_web_bridge')
        self._pub = self.create_publisher(Joy, '/joy', 10)

    def publish_from_json(self, data):
        axes = data.get('axes', [])
        buttons = data.get('buttons', [])
        if len(axes) != AXES_LEN or len(buttons) != BUTTONS_LEN:
            self.get_logger().warn(
                f"Ignoring malformed /joy message: {len(axes)} axes, "
                f"{len(buttons)} buttons (expected {AXES_LEN}/{BUTTONS_LEN})"
            )
            return
        msg = Joy()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self._pub.publish(msg)


class JoyWebSocketHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, node):
        self._node = node

    def check_origin(self, origin):
        return True  # served on a local/lab network — any origin is fine

    def on_message(self, message):
        try:
            data = json.loads(message)
        except ValueError:
            self._node.get_logger().warn(f"Ignoring non-JSON /joy message: {message!r}")
            return
        self._node.publish_from_json(data)


def make_app(node, web_path):
    return tornado.web.Application([
        (r'/joy', JoyWebSocketHandler, dict(node=node)),
        (r'/(.*)', tornado.web.StaticFileHandler,
         dict(path=web_path, default_filename='index.html')),
    ])


def main(args=None):
    web_path = sys.argv[1] if len(sys.argv) > 1 else _default_web_path()
    http_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_HTTP_PORT
    ws_port = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_WS_PORT

    rclpy.init(args=args)
    node = JoyWebBridge()

    app = make_app(node, web_path)
    app.listen(http_port)
    if ws_port != http_port:
        app.listen(ws_port)

    io_loop = tornado.ioloop.IOLoop.current()
    server_thread = threading.Thread(target=io_loop.start, daemon=True)
    server_thread.start()

    node.get_logger().info(f"Serving '{web_path}' at http://0.0.0.0:{http_port}/")
    node.get_logger().info(f"Accepting /joy WebSocket connections at ws://0.0.0.0:{ws_port}/joy")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        io_loop.add_callback(io_loop.stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
