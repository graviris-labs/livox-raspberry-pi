#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source the workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
    echo "Workspace sourced successfully"
else
    echo "WARNING: Workspace setup.bash not found"
    # Try to find it elsewhere
    SETUP_FILE=$(find /ros2_ws -name "setup.bash" | grep -v "/opt/ros" | head -1)
    if [ -n "$SETUP_FILE" ]; then
        source $SETUP_FILE
        echo "Found and sourced workspace at $SETUP_FILE"
    else
        echo "ERROR: Could not find workspace setup.bash"
        # Try to build the workspace
        cd /ros2_ws
        colcon build --symlink-install
        if [ -f "/ros2_ws/install/setup.bash" ]; then
            source /ros2_ws/install/setup.bash
            echo "Workspace built and sourced successfully"
        else
            echo "ERROR: Failed to build workspace"
            exit 1
        fi
    fi
fi

# Check if the package exists
echo "Checking ROS2 packages..."
ros2 pkg list | grep livox

# Check available launch files
echo "Available launch files:"
find /ros2_ws -name "*.py" | grep -i launch

# Run Livox driver
echo "Running Livox driver..."
cd /ros2_ws
# Try to find an appropriate launch file
LAUNCH_FILE=$(find /ros2_ws -name "*launch.py" | grep -i livox | head -1)
if [ -n "$LAUNCH_FILE" ]; then
    echo "Found launch file: $LAUNCH_FILE"
    ros2 launch $LAUNCH_FILE
else
    echo "No launch file found. Starting a bash shell for manual control."
    exec bash
fi
