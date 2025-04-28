#!/bin/bash

# This script configures the network for Livox LiDAR connection

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

# Define variables
INTERFACE="eth0"
IP_ADDRESS="192.168.1.50/24"
CONFIG_FILE="/etc/dhcpcd.conf"

# Backup original config
cp $CONFIG_FILE ${CONFIG_FILE}.backup

# Check if configuration already exists
if grep -q "^interface $INTERFACE" $CONFIG_FILE; then
  echo "Network configuration for $INTERFACE already exists"
  echo "Please check $CONFIG_FILE manually"
  exit 1
fi

# Add network configuration
cat << EOF >> $CONFIG_FILE

# Livox LiDAR static IP configuration
interface $INTERFACE
static ip_address=$IP_ADDRESS
EOF

echo "Network configuration added to $CONFIG_FILE"
echo "Restarting networking service..."

# Restart networking
service dhcpcd restart

echo "Network configuration complete"
echo "Your Raspberry Pi's static IP is now set to ${IP_ADDRESS%/*}"
echo "Please make sure your Livox LiDAR is on the same network"