#!/usr/bin/env python3

import rospy
import json
import cv2
import time
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys

bridge = CvBridge()
mqtt_client = None

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to MQTT broker for IR")
    else:
        rospy.logerr(f"Failed to connect to MQTT broker (IR) with code {rc}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        rospy.logwarn(f"IR MQTT disconnected. Code: {rc}")

def image_callback(msg):
    try:
        # Convert ROS Image to OpenCV
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        # Encode as JPEG
        ret, jpeg = cv2.imencode('.jpg', cv_img)
        if not ret:
            rospy.logwarn("Failed to encode JPEG")
            return

        payload = {
            "timestamp": msg.header.stamp.to_sec(),
            "frame_id": msg.header.frame_id,
            "image": jpeg.tobytes().hex()  # send as hex string
        }

        mqtt_client.publish("ir/image", json.dumps(payload))

    except Exception as e:
        rospy.logerr(f"Error in IR image callback: {str(e)}")

def main():
    global mqtt_client
    rospy.init_node("ir_mqtt_publisher", anonymous=True)

    # Get MQTT broker settings
    mac_mini_ip = "192.168.50.49"  # <- change if needed
    mqtt_broker = rospy.get_param('~mqtt_broker', mac_mini_ip)
    mqtt_port = rospy.get_param('~mqtt_port', 1883)
    
    rospy.loginfo(f"Attempting to connect to MQTT broker at {mqtt_broker}:{mqtt_port}")
    
    # Create MQTT client (with compatibility for various paho-mqtt versions)
    try:
        # Basic client creation with minimal arguments
        mqtt_client = mqtt.Client("ir_publisher")
    except Exception as e:
        rospy.logerr(f"Failed to create MQTT client: {e}")
        return
    
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    
    # Try to connect with retry
    max_retries = 5
    for retry in range(max_retries):
        try:
            rospy.loginfo(f"MQTT connection attempt {retry+1}/{max_retries}")
            mqtt_client.connect(mqtt_broker, mqtt_port, 60)
            mqtt_client.loop_start()
            break
        except Exception as e:
            rospy.logerr(f"Failed to connect to MQTT broker (attempt {retry+1}): {e}")
            if retry == max_retries - 1:
                rospy.logerr("Max retries reached. Cannot connect to MQTT broker.")
                return
            rospy.sleep(5)  # Wait before retry

    # Update this topic to match your camera's topic
    camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
    rospy.loginfo(f"Subscribing to camera topic: {camera_topic}")
    rospy.Subscriber(camera_topic, Image, image_callback, queue_size=1)

    rospy.spin()
    
    if mqtt_client:
        try:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        except:
            pass

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Fatal error in IR publisher: {e}")
        sys.exit(1)
