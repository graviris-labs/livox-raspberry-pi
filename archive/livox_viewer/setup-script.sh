#!/bin/bash

# Setup script for Livox 360 LiDAR with Raspberry Pi
# This script will:
# 1. Install Docker and Docker Compose
# 2. Configure network settings
# 3. Create necessary directories
# 4. Build and run the Docker container

echo "Setting up Livox 360 LiDAR with Raspberry Pi..."

# Update system packages
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install Docker dependencies
echo "Installing dependencies..."
sudo apt install -y apt-transport-https ca-certificates curl gnupg lsb-release

# Install Docker
echo "Installing Docker..."
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
rm get-docker.sh

# Add current user to Docker group
echo "Adding user to Docker group..."
sudo usermod -aG docker $USER

# Set up directories
echo "Setting up directories..."
mkdir -p config data

# Configure network (optional - uncomment if needed)
# echo "Configuring network..."
# sudo tee -a /etc/dhcpcd.conf > /dev/null << EOT
# 
# # Livox LiDAR static IP configuration
# interface eth0
# static ip_address=192.168.1.50/24
# EOT

echo "Setup complete!"
echo "Please log out and log back in for group changes to take effect."
echo "Then run 'docker compose up -d' to start the Livox container."