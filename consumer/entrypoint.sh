#!/bin/bash
set -e

echo "Starting MQTT to SQLite consumer"
echo "MQTT_BROKER=${MQTT_BROKER}"
echo "MQTT_PORT=${MQTT_PORT}"

# Create data directory if it doesn't exist
mkdir -p /data

# Start the MQTT consumer
echo "Starting MQTT consumer..."
exec python3 /app/mqtt_consumer.py \
  --broker "${MQTT_BROKER}" \
  --port "${MQTT_PORT}" \
  --database "/data/livox_data.db"
