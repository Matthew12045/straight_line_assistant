#!/usr/bin/env bash
# apply_gains.sh — apply staged PID gains to /straight_line_assistant (live, no restart)
# Usage:  ./apply_gains.sh current|stage1|stage2|stage3
# Run on the ROS 2 machine while the node is running.

set -euo pipefail
NODE=/straight_line_assistant

case "${1:-}" in
  current)  KP=1.2; KI=0.01; KD=0.4;  ILIM=1.0 ;;   # as-shipped params.yaml
  stage1)   KP=0.8; KI=0.10; KD=0.0;  ILIM=1.0 ;;   # safe start
  stage2)   KP=1.4; KI=0.30; KD=0.1;  ILIM=1.0 ;;   # target (PM ~65 deg)
  stage3)   KP=1.7; KI=0.50; KD=0.2;  ILIM=1.0 ;;   # crisp  (PM ~55 deg)
  *) echo "usage: $0 current|stage1|stage2|stage3"; exit 1 ;;
esac

echo "Applying $1: kp=$KP ki=$KI kd=$KD integral_limit=$ILIM"
ros2 param set $NODE kp "$KP"
ros2 param set $NODE ki "$KI"
ros2 param set $NODE kd "$KD"
ros2 param set $NODE integral_limit "$ILIM"
echo "Done. Verify with: ros2 param get $NODE kp"
echo "Reminder: ceilings kp<=2.2 ki<=0.8 kd<=0.3 ; one knob per test run ; export a CSV each run."
