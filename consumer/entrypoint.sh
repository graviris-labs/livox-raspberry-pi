#!/bin/bash
set -e

echo "Starting MQTT to SQLite consumer"
echo "BROKER_ADDRESS=${BROKER_ADDRESS}"
echo "BROKER_PORT=${BROKER_PORT}"

# Create data directory if it doesn't exist
mkdir -p /data

# Start the MQTT consumer
echo "Starting MQTT consumer..."
exec python3 /app/mqtt_consumer.py
