#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import time
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('mqtt-consumer-test')

# Configuration
BROKER_ADDRESS = "192.168.50.191"  # Your Raspberry Pi's IP
PORT = 1883
TOPIC = "livox/+"  # Subscribe to all livox topics

# Callback when connecting
def on_connect(client, userdata, flags, rc):
    logger.info(f"Connected with result code {rc}")
    client.subscribe(TOPIC)
    logger.info(f"Subscribed to topic: {TOPIC}")

# Callback when receiving a message
def on_message(client, userdata, msg):
    logger.info(f"Received message on topic {msg.topic}: {len(msg.payload)} bytes")

def main():
    # Create client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    # Connect with retry logic
    connected = False
    retry_count = 0
    max_retries = 10
    
    while not connected and retry_count < max_retries:
        try:
            logger.info(f"Attempting to connect to broker at {BROKER_ADDRESS}:{PORT} (attempt {retry_count+1})")
            client.connect(BROKER_ADDRESS, PORT, 60)
            connected = True
            logger.info("Connected!")
        except Exception as e:
            retry_count += 1
            logger.error(f"Connection failed: {e}")
            time.sleep(2)
    
    if not connected:
        logger.error("Failed to connect to MQTT broker after multiple attempts")
        return 1
    
    try:
        # Start loop
        client.loop_forever()
    except KeyboardInterrupt:
        logger.info("Exiting...")
    
    return 0

if __name__ == "__main__":
    exit(main())