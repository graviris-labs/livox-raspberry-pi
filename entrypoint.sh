#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Debug information
echo "Checking ROS2 packages..."
ros2 pkg list | grep livox

# Source the workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
    echo "Workspace sourced successfully"
else
    echo "WARNING: Workspace setup.bash not found at expected location"
    # Try to find it elsewhere
    SETUP_FILE=$(find /ros2_ws -name "setup.bash" | head -1)
    if [ -n "$SETUP_FILE" ]; then
        source $SETUP_FILE
        echo "Found and sourced workspace at $SETUP_FILE"
    else
        echo "ERROR: Could not find workspace setup.bash"
        exit 1
    fi
fi

# Check if the package exists now
echo "Checking ROS2 packages after sourcing workspace..."
ros2 pkg list | grep livox

# Check available launch files
echo "Available launch files:"
find /ros2_ws -name "*.py" | grep -i launch

# Run Livox driver with appropriate launch file
echo "Running Livox
