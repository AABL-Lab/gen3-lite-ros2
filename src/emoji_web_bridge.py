#!/usr/bin/env python3
"""
emoji_web_bridge.py — serves the browser emoji action panel
(emojis/index.html) and forwards its button clicks straight to
/emoji_action, without going through rosbridge_server.

Same rationale as joy_web_bridge.py: rosbridge_server's build in this
workspace fails to start (see joy_web_bridge.py's docstring / README for
details), so this talks to the browser directly with a small
self-contained WebSocket server (tornado) instead — no rosbridge/rosapi
involved, and no external CDN dependency in the page either.

Serves:
    http://<host>:<http_port>/            static files from <web_path>
    ws://<host>:<ws_port>/emoji_action     the page sends one JSON object
                                            per message: {"action_id": "..."}.
                                            Each is published verbatim to
                                            /emoji_action, no server-side
                                            state.

Publishes:
    /emoji_action                          (std_msgs/String)

Usage:
    ros2 run gen3-lite-ros2 emoji_web_bridge [web_path] [http_port] [ws_port]

    web_path   directory to serve static files from
               (default: this package's emojis/ directory)
    http_port  default 8001
    ws_port    default 9091 (distinct from joy_web_bridge's 8000/9090 so
               both web interfaces can run at the same time)

Note: nothing in this package currently subscribes to /emoji_action —
this script only provides the panel and the topic. Wire up a listener
(e.g. to trigger recorded gestures) separately if you need one.
"""
import json
import sys
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

import tornado.ioloop
import tornado.web
import tornado.websocket


def _default_web_path():
    # Falls back to the installed copy of emojis/ under this package's
    # share directory. launch/web_interface_emojis.launch.py instead
    # defaults to the source tree directly, so edits to index.html don't
    # need a rebuild when launched that way.
    return get_package_share_directory('gen3-lite-ros2') + '/emojis'


DEFAULT_HTTP_PORT = 8001
DEFAULT_WS_PORT = 9091


class EmojiWebBridge(Node):
    """Publishes /emoji_action from whatever JSON the browser's WebSocket sends."""

    def __init__(self):
        super().__init__('emoji_web_bridge')
        self._pub = self.create_publisher(String, '/emoji_action', 10)

    def publish_from_json(self, data):
        action_id = data.get('action_id')
        if not isinstance(action_id, str) or not action_id:
            self.get_logger().warn(f"Ignoring malformed /emoji_action message: {data!r}")
            return
        self._pub.publish(String(data=action_id))
        self.get_logger().info(f"Published action: {action_id}")


class EmojiWebSocketHandler(tornado.websocket.WebSocketHandler):
    def initialize(self, node):
        self._node = node

    def check_origin(self, origin):
        return True  # served on a local/lab network — any origin is fine

    def on_message(self, message):
        try:
            data = json.loads(message)
        except ValueError:
            self._node.get_logger().warn(f"Ignoring non-JSON /emoji_action message: {message!r}")
            return
        self._node.publish_from_json(data)


def make_app(node, web_path):
    return tornado.web.Application([
        (r'/emoji_action', EmojiWebSocketHandler, dict(node=node)),
        (r'/(.*)', tornado.web.StaticFileHandler,
         dict(path=web_path, default_filename='index.html')),
    ])


def main(args=None):
    web_path = sys.argv[1] if len(sys.argv) > 1 else _default_web_path()
    http_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_HTTP_PORT
    ws_port = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_WS_PORT

    rclpy.init(args=args)
    node = EmojiWebBridge()

    app = make_app(node, web_path)
    app.listen(http_port)
    if ws_port != http_port:
        app.listen(ws_port)

    io_loop = tornado.ioloop.IOLoop.current()
    server_thread = threading.Thread(target=io_loop.start, daemon=True)
    server_thread.start()

    node.get_logger().info(f"Serving '{web_path}' at http://0.0.0.0:{http_port}/")
    node.get_logger().info(f"Accepting /emoji_action WebSocket connections at ws://0.0.0.0:{ws_port}/emoji_action")

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
