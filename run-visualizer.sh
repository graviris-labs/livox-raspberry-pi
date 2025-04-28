#!/bin/bash

# This script sets up and runs the Livox LiDAR visualizer on Raspberry Pi

# Allow X server to accept connections from any host
xhost +local:

# Choose which visualizer to use
echo "Which visualizer would you like to use?"
echo "1) C++ PCL Visualizer (better performance, more complex)"
echo "2) Python Matplotlib Visualizer (simpler, lighter)"
read -p "Enter choice [1/2]: " choice

if [ "$choice" = "1" ]; then
    echo "Using C++ PCL Visualizer"
    DOCKERFILE="Dockerfile.visualizer"
elif [ "$choice" = "2" ]; then
    echo "Using Python Matplotlib Visualizer"
    DOCKERFILE="Dockerfile.python-visualizer"
else
    echo "Invalid choice. Defaulting to C++ PCL Visualizer"
    DOCKERFILE="Dockerfile.visualizer"
fi

# Create a temporary Docker Compose file
cat > docker-compose.viz.yml << EOF
version: '3'
services:
  livox:
    build: .
    container_name: livox_lidar
    restart: unless-stopped
    network_mode: "host"
    volumes:
      - ./config:/opt/livox_config
      - ./data:/opt/livox_data
  
  visualizer:
    build:
      context: .
      dockerfile: $DOCKERFILE
    container_name: livox_visualizer
    restart: unless-stopped
    network_mode: "host"
    environment:
      - DISPLAY=$DISPLAY
    volumes:
      - ./config:/opt/livox_config
      - ./data:/opt/livox_data
      - /tmp/.X11-unix:/tmp/.X11-unix
EOF

# Stop any existing containers
docker compose down

# Start the containers with the visualization
docker compose -f docker-compose.viz.yml up -d

# Show container status
docker ps

# Show logs from the visualizer
echo "Showing logs from the visualizer. Press Ctrl+C to exit."
docker logs -f livox_visualizer