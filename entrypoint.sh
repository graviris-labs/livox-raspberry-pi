#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Print debug information
echo "PATH: $PATH"
which ros2 || echo "ros2 command not found"

# Execute command
exec "$@"
