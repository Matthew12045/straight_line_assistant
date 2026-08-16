# straight_line_assistant

A single ROS 2 node that combines **TurtleBot3 keyboard teleop** with a
**PID heading-hold controller**.  No external teleop node is needed —
just run this one node, drive with `w/a/s/d/x`, and the PID
automatically keeps you on a straight heading whenever you're driving
forward/backward without turning.

## How it works
- A **background thread** reads keyboard input using the standard
  TurtleBot3 key layout (`w`/`x` = increase/decrease linear velocity,
  `a`/`d` = increase/decrease angular velocity, `s`/space = stop).
- A **20 Hz timer** runs the control loop, smoothly ramps velocities,
  applies PID heading correction when going straight, and publishes to
  `/cmd_vel` every tick — so the robot always receives a steady command
  stream regardless of how fast or slow you press keys.
- Velocity limits are read from the `TURTLEBOT3_MODEL` environment
  variable (defaults to `burger`: 0.22 m/s, 2.84 rad/s).

## Topics
- Subscribes: `/odometry/filtered` (nav_msgs/Odometry)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)
- The launch file starts the custom `ekf_node`, which is the only intended publisher of `/odometry/filtered`; do not also run `robot_localization` with this launch file.

## Requirements
- ROS 2 (Humble, Iron, or Jazzy — no version-specific APIs are used)
- `rclpy`, `geometry_msgs`, `nav_msgs` (come with a standard ROS 2 install)
- `TURTLEBOT3_MODEL` environment variable set (e.g. `burger`, `waffle`, `waffle_pi`)

## Build
```bash
cd ~/ros2_ws
cp -r /path/to/straight_line_assistant src/
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select straight_line_assistant
source install/setup.bash
```

## Run
**Must be run in a terminal** (needs keyboard access):
```bash
export TURTLEBOT3_MODEL=burger   # if not already set
ros2 run straight_line_assistant straight_line_assistant_node
```

Or with the provided launch file (loads `config/params.yaml`):
```bash
ros2 launch straight_line_assistant straight_line_assistant.launch.py
```

## Controls
```
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (step: 0.01 m/s)
a/d : increase/decrease angular velocity (step: 0.1 rad/s)
space / s : force stop
CTRL-C : quit

Heading-hold PID engages automatically when driving straight.
```

## Tuning
All PID/timeout values are ROS 2 parameters and can be retuned live:
```bash
ros2 param set /straight_line_assistant kp 2.0
```
or edited ahead of time in `config/params.yaml`.

| Parameter         | Default | Description                           |
|-------------------|---------|---------------------------------------|
| `kp`              | 1.5     | Proportional gain                     |
| `ki`              | 0.01    | Integral gain                         |
| `kd`              | 0.2     | Derivative gain                       |
| `integral_limit`  | 1.0     | Anti-windup clamp                     |
| `angular_epsilon` | 0.001   | "No turn" threshold (rad/s)           |
| `odom_timeout`    | 0.5     | Stale odometry threshold (seconds)    |
| `control_rate`    | 20.0    | Control loop frequency (Hz)           |

## Package layout
```
straight_line_assistant/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/straight_line_assistant
├── straight_line_assistant/
│   ├── __init__.py
│   └── straight_line_assistant_node.py
├── launch/straight_line_assistant.launch.py
└── config/params.yaml
```
