#!/usr/bin/env python3

import rospy
import json
import cv2
import time
import paho.mqtt.client as mqtt
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

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

    mac_mini_ip = "192.168.50.49"  # <- change if needed
    mqtt_broker = rospy.get_param('~mqtt_broker', mac_mini_ip)
    mqtt_port = rospy.get_param('~mqtt_port', 1883)

    # Fix for paho-mqtt 2.0+ compatibility
    try:
        # For paho-mqtt 2.0+
        mqtt_client = mqtt.Client("ir_publisher", callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
    except (AttributeError, TypeError):
        # Fallback for older versions
        mqtt_client = mqtt.Client("ir_publisher", protocol=mqtt.MQTTv311)
    
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.connect(mqtt_broker, mqtt_port, 60)
    mqtt_client.loop_start()

    # Update this topic to match your camera's topic
    camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
    rospy.loginfo(f"Subscribing to camera topic: {camera_topic}")
    rospy.Subscriber(camera_topic, Image, image_callback, queue_size=1)

    rospy.spin()
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

if __name__ == "__main__":
    main()
