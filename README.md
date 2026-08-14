# straight_line_assistant

A ROS 2 node that helps a teleop-driven robot hold a straight heading.
When the driver commands forward/backward motion with no intentional
turn, the node locks the current yaw (from `/odometry/filtered`) and
uses a PID loop to correct drift, publishing the corrected command to
`/cmd_vel`. If the driver turns, or if teleop/odom data goes stale,
commands pass straight through.

## Topics
- Subscribes: `/teleop/cmd_vel` (geometry_msgs/Twist), `/odometry/filtered` (nav_msgs/Odometry)
- Publishes: `/cmd_vel` (geometry_msgs/Twist)

## Requirements
- ROS 2 (Humble, Iron, or Jazzy — no version-specific APIs are used)
- `rclpy`, `geometry_msgs`, `nav_msgs` (come with a standard ROS 2 install)
- `launch`, `launch_ros`, `ament_index_python` (only needed to use the
  provided launch file; otherwise optional)

## Build
Unzip this into the `src/` folder of a colcon workspace, then from the
workspace root:

```bash
cd ~/ros2_ws          # or wherever your workspace root is
cp -r /path/to/straight_line_assistant src/
rosdep install --from-paths src --ignore-src -r -y   # installs any missing deps
colcon build --packages-select straight_line_assistant
source install/setup.bash
```

## Run
Directly:
```bash
ros2 run straight_line_assistant straight_line_assistant_node
```

Or with the provided launch file (loads `config/params.yaml`):
```bash
ros2 launch straight_line_assistant straight_line_assistant.launch.py
```

## Tuning
All PID/timeout values are ROS 2 parameters, so they can be retuned
live without restarting:
```bash
ros2 param set /straight_line_assistant kp 2.0
```
or edited ahead of time in `config/params.yaml`.

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
