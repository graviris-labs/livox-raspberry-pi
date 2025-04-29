#!/bin/bash

# Stop and remove any existing container
docker stop livox_ros2_driver 2>/dev/null || true
docker rm livox_ros2_driver 2>/dev/null || true

# Build the Docker image
docker build -t livox_ros2_image .

# Create the config directory if it doesn't exist
mkdir -p config

# Run the Docker container
docker run --network host \
  --name livox_ros2_driver \
  -v $(pwd)/config:/ros2_ws/src/livox_ros_driver2/config \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  -it livox_ros2_image