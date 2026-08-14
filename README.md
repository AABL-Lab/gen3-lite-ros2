For additional information, reference the doc here: https://docs.google.com/document/d/1e49sWaCcWIZYFpPIl9jcipFpu3DYWyUefIVDwcwyToA/edit?usp=sharing 

# gen3-lite-ros2

ROS 2 package for driving a Kinova Gen3 Lite arm. Includes gamepad teleoperation (`joy_teleop.py`) for the
arm's Cartesian twist controller and gripper — controllable from either a
physical gamepad or a browser-based on-screen controller — waypoint
playback (`joint_playback.py`) for replaying a fixed sequence of joint
positions, two recorders for building those waypoint files
(`joint_recorder.py`, keyboard-driven; `behavior_recorder.py`,
Xbox-controller-driven), and a browser-based emoji action panel
(`emoji_web_bridge.py`) that publishes named actions to `/emoji_action`.

## Build

From the workspace root:

```bash
colcon build --packages-select gen3-lite-ros2
source install/setup.bash
```

## Quick start: full robot + gamepad teleop

`launch/xbox.launch.py` brings up the real Gen3 Lite arm (via
`kortex_bringup`), switches the controller manager over to
`twist_controller`, and starts `joy_teleop`:

```bash
ros2 launch gen3-lite-ros2 xbox.launch.py controller:=xbox
```

- `controller:=xbox` (default) — also launches `joy_node` so a wired/Bluetooth
  Xbox controller plugged into this machine publishes `/joy`.
- `controller:=web` — skips `joy_node`; use this with the browser-based web
  interface below (or any other bridge that publishes `/joy` itself).

The arm's IP is hardcoded in `xbox.launch.py` (`robot_ip: 192.168.1.10`) —
edit it there if your robot is at a different address.

## Running joy_teleop by itself

If the robot and controllers are already up (e.g. you only want to restart
teleop), run the node directly:

```bash
ros2 run gen3-lite-ros2 joy_teleop xbox   # or: joy_teleop web
```

The argument selects the same `xbox`/`web` behavior described above and
defaults to `web` if omitted.

## Web interface (browser-based teleop)

`arrows/index.html` is an on-screen arrow-button + keyboard controller
(translate/rotate toggle, gripper open/close) — no physical gamepad
needed. `launch/web_interface_arrows.launch.py` runs `joy_web_bridge.py`,
which serves the page and accepts its WebSocket connection directly,
publishing whatever it sends straight to `/joy`:

```bash
ros2 launch gen3-lite-ros2 xbox.launch.py controller:=web        # in one terminal
ros2 launch gen3-lite-ros2 web_interface_arrows.launch.py        # in another
```

Then open `http://<this machine's address>:8000` in a browser (phone,
tablet, laptop — anything on the same network). The page connects back to
`joy_web_bridge` on port 9090 on whatever host served it, so no IP needs
to be hardcoded in the page itself.

**Why not rosbridge?** `web_interface_arrows.launch.py` originally used
`rosbridge_server` + `roslib.js`. In this workspace, `rosbridge_server`'s
build currently fails to start — `rosbridge_library` imports
`rosidl_pycommon.interface_base_classes`, which the installed
`rosidl_pycommon` doesn't have (a real version mismatch between the pinned
`rosbridge_suite` checkout in `src/rosbridge_suite` and this ROS distro,
confirmed *not* fixed by upgrading `rosidl_pycommon`). Rather than change
the pinned version of `rosbridge_suite` or `rosidl_pycommon`,
`joy_web_bridge.py` talks to the browser with a small self-contained
WebSocket server (`tornado`, already installed as a transitive
dependency) — no `rosbridge`/`rosapi` involved, and the page no longer
depends on the `roslib.js` CDN either.

Controls: on-screen arrows (or arrow keys) move X/Y or Z depending on
mode, `Translate`/`Rotate` buttons (or `M`) toggle mode, and the gripper
buttons (or `O`/`C`) open/close it — see `arrows/index.html` for the exact
`/joy` values it sends, which are built to match `joy_teleop.py`'s default
`AXES`/`BUTTONS`/`ACTIONS`. If you customize that mapping in
`joy_teleop.py`, update the `IDX`/`BTN` constants near the top of
`arrows/index.html`'s script to match.

`web_path` (default: this package's `arrows/` directory) and the ports
(8000 for the page, 9090 for the WebSocket) are hardcoded for a
single-machine dev setup — override them via launch arguments
(`web_path:=`, `http_port:=`, `ws_port:=`), or run
`ros2 run gen3-lite-ros2 joy_web_bridge [web_path] [http_port] [ws_port]`
directly, if you need something else.

## Emoji action panel

`emojis/index.html` is a grid of buttons ("Robot Action Panel") that each
publish an `action_id` string (e.g. `thumbs_up`, `laugh`, `shocked`) to
`/emoji_action` when clicked. It works the same way as the arrows
interface above — `emoji_web_bridge.py` serves the page and forwards its
WebSocket messages straight to the topic, no `rosbridge` involved:

```bash
ros2 launch gen3-lite-ros2 web_interface_emojis.launch.py
```

Then open `http://<this machine's address>:8001` in a browser. It uses
different default ports (8001/9091) than the arrows interface (8000/9090)
so both can run at the same time.

**Nothing subscribes to `/emoji_action` yet** — this only provides the
panel and the topic. (AZ_demo's version wires `handshake` up to replaying
a single hardcoded trajectory file for its 7-DOF arm; that's a rougher,
more coupled prototype that doesn't map cleanly onto Gen3 Lite's 6 joints,
so it wasn't ported here. `joint_playback.py`/`joint_recorder.py` above
are the general-purpose way to record and replay a gesture on this arm —
wiring a specific `action_id` to a specific waypoint file via a small
listener node is a natural follow-up if you want that.)

### Topics used by emoji_web_bridge

| Name | Type | Direction |
|---|---|---|
| `/emoji_action` | `std_msgs/String` | published |

## Controls

| Control | Action |
|---|---|
| Hold **RB** | enable arm movement (release to stop) |
| Hold **LB** + RB | turbo speed |
| **Y** | toggle translate / rotate mode |
| Left stick / right stick vertical (TRANSLATE mode) | move X / Y / Z |
| Left stick / right stick horizontal (ROTATE mode) | roll / pitch / yaw |
| **LT** | open gripper |
| **RT** | close gripper |

Movement is interpreted in the robot's base frame and rotated into the
end-effector frame via TF, so "push stick forward" always moves the arm
away from the base regardless of wrist orientation.

### Customizing the mapping

All of the above is configurable at the top of
[`src/joy_teleop.py`](src/joy_teleop.py) without touching any control logic:

- **`AXES` / `BUTTONS`** — physical index each named axis/button is reported
  at by the gamepad driver. Edit if a non-Xbox-layout controller reports
  indices differently.
- **`ACTIONS`** — which button/trigger drives each command (`enable_move`,
  `turbo`, `toggle_mode`, `gripper_open`, `gripper_close`). Change a value to
  move a command to a different button.
- **`AXIS_MAP`** — which stick axis drives each Cartesian DOF
  (`translate_x/y/z`, `rotate_roll/pitch/yaw`), plus a sign to flip its
  direction.
- **Speed/behavior constants** — `LINEAR_SCALE`, `ANGULAR_SCALE`,
  `TURBO_MULT`, `PUBLISH_HZ`, `TRIGGER_THRESHOLD`, `GRIPPER_*`.

The node's startup log prints the active mapping, so a remap is reflected
there automatically.

## Topics & actions used by joy_teleop

| Name | Type | Direction |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | subscribed |
| `/twist_controller/commands` | `geometry_msgs/Twist` | published (at `PUBLISH_HZ`, always — zeros when movement isn't enabled, to keep the kortex driver's heartbeat alive) |
| `/robotiq_gripper_controller/gripper_cmd` | `control_msgs/action/ParallelGripperCommand` | action client |

## Joint waypoint playback

`joint_playback.py` reads a text file of joint-angle waypoints and sends
each one to `joint_trajectory_controller` as a `FollowJointTrajectory`
goal, in order, waiting for each to finish before sending the next.

This needs `joint_trajectory_controller` active, which is `kortex_bringup`'s
default — but note `xbox.launch.py` switches the arm to `twist_controller`
for teleop, so if the arm was last brought up for teleop, switch back first:

```bash
ros2 control switch_controllers \
    --activate joint_trajectory_controller --deactivate twist_controller
```

### Quick start: full robot + playback

```bash
ros2 launch gen3-lite-ros2 joint_playback.launch.py waypoints_file:=config/waypoints/example.txt
```

This brings up the arm with `joint_trajectory_controller` active (no
switching needed) and plays back the given file. `waypoints_file` is
required; `robot_ip` defaults to `192.168.1.10` like `xbox.launch.py`.

### Running joint_playback by itself

If the robot is already up under `joint_trajectory_controller`:

```bash
ros2 run gen3-lite-ros2 joint_playback config/waypoints/example.txt
```

### Waypoint file format

One waypoint per line, whitespace- or comma-separated:

```
joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 [duration]
```

- The six joint angles are in radians, in `JOINT_NAMES` order (see
  [`src/joint_playback.py`](src/joint_playback.py)).
- The optional trailing `duration` is how many seconds the controller
  should take to reach that waypoint from the previous one. If omitted,
  `DEFAULT_TIME_PER_WAYPOINT` (4s) is used.
- Blank lines are skipped; `#` starts a comment.

See [`config/waypoints/example.txt`](config/waypoints/example.txt) for a
worked example. **The angles in that file are illustrative only** — verify
against your workspace and the arm's joint limits before running on real
hardware.

### Topics & actions used by joint_playback

| Name | Type | Direction |
|---|---|---|
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | action client |

## Recording waypoints

`joint_recorder.py` builds a waypoint file interactively: jog the arm to a
pose by whatever means is running (`joy_teleop.py`, MoveIt, hand-backdriving
if the arm supports it), hit Enter in the recorder's terminal, and it
appends the arm's current joint positions to the output file in exactly
the format `joint_playback.py` expects — no manual transcription.

It only subscribes to `/joint_states`, so it doesn't care which controller
is currently active and can run alongside `joy_teleop.py` or any other way
of moving the arm.

```bash
ros2 run gen3-lite-ros2 joint_recorder config/waypoints/my_trajectory.txt
```

An optional second argument sets the per-waypoint duration written into
each recorded line (default 4s):

```bash
ros2 run gen3-lite-ros2 joint_recorder config/waypoints/my_trajectory.txt 2.0
```

If `<output_file>` already exists, new waypoints are appended after its
existing contents rather than overwriting it.

At the `>` prompt:

| Input | Action |
|---|---|
| *(blank)* Enter | record the arm's current pose as the next waypoint |
| `u` | undo (remove) the last waypoint recorded **this session** |
| `q` | quit |

The file is rewritten after every record/undo, so it's always safe to feed
to `joint_playback.py` even if you exit without typing `q`. When you're
done recording, play it back with:

```bash
ros2 run gen3-lite-ros2 joint_playback config/waypoints/my_trajectory.txt
```

(with `joint_trajectory_controller` active — see above).

### Topics used by joint_recorder

| Name | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | subscribed |

## Recording a behavior with the Xbox controller

`behavior_recorder.py` is `joint_recorder.py`'s controller-driven sibling:
instead of hitting Enter at a keyboard, you tap a button on the Xbox
controller to record a waypoint — so you never have to let go of the
sticks while jogging the arm. Run it **alongside** `joy_teleop.py`, which
is what actually drives the arm; this node only watches `/joy` for its
own buttons (chosen to not collide with `joy_teleop.py`'s RB/LB/Y/LT/RT)
and `/joint_states` for the current pose:

```bash
ros2 launch gen3-lite-ros2 xbox.launch.py controller:=xbox                      # in one terminal
ros2 run gen3-lite-ros2 behavior_recorder config/waypoints/my_behavior.txt      # in another
```

An optional second argument sets the per-waypoint duration, same as
`joint_recorder.py` (default 4s). `<output_file>` is appended to if it
already exists.

Controls (Xbox buttons — edit `RECORD_ACTIONS` near the top of
`behavior_recorder.py` to remap):

| Button | Action |
|---|---|
| `A` | record the arm's current pose as the next waypoint |
| `B` | undo (remove) the last waypoint recorded **this session** |
| `start` | stop recording (same as Ctrl+C) |

Same live-rewrite guarantee as `joint_recorder.py`: the file is updated
after every record/undo, so it's always safe to hand to
`joint_playback.py`, and the waypoint format is identical (interchangeable
with files from `joint_recorder.py`). Once you're happy with the
behavior, replay it with:

```bash
ros2 run gen3-lite-ros2 joint_playback config/waypoints/my_behavior.txt
```

(with `joint_trajectory_controller` active instead of `twist_controller`
— see "Joint waypoint playback" above; you'll need to switch controllers
after recording, since `xbox.launch.py` runs `twist_controller` for
teleop.)

### Topics used by behavior_recorder

| Name | Type | Direction |
|---|---|---|
| `/joy` | `sensor_msgs/Joy` | subscribed |
| `/joint_states` | `sensor_msgs/JointState` | subscribed |

## Dependencies

`rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`, `control_msgs`,
`trajectory_msgs`, `tf2_ros`, `python3-numpy`, `joy`, `python3-tornado`
(for the web interfaces' WebSocket servers), plus `kortex_bringup` and
`kinova_gen3_lite_moveit_config` for the full launch-file bring-up.
See `package.xml` for the complete list.
