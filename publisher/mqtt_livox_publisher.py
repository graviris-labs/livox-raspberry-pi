#!/usr/bin/env python3

import rospy
import json
import numpy as np
import paho.mqtt.client as mqtt
import struct
import time
from std_msgs.msg import Header
from sensor_msgs.msg import Imu

# Import livox ROS message formats if available, otherwise define our own
try:
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
except ImportError:
    rospy.logwarn("Could not import livox_ros_driver2 messages. Using standard message types.")
    # Will use standard PointCloud2 as fallback

# Global MQTT client
mqtt_client = None

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        rospy.loginfo("Connected to MQTT broker")
    else:
        rospy.logerr(f"Failed to connect to MQTT broker with code {rc}")

def on_disconnect(client, userdata, rc):
    if rc != 0:
        rospy.logwarn(f"Unexpected MQTT disconnection. Will auto-reconnect. Code: {rc}")

def imu_callback(msg):
    """Callback for IMU messages"""
    try:
        # Create a simplified JSON structure for IMU data
        imu_data = {
            "header": {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w
            },
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z
            }
        }
        
        # Publish to MQTT
        mqtt_client.publish("livox/imu", json.dumps(imu_data))
        
    except Exception as e:
        rospy.logerr(f"Error in IMU callback: {str(e)}")

def custom_msg_callback(msg):
    """Callback for Livox CustomMsg messages"""
    try:
        # Only process every 5th message to reduce network load
        custom_msg_callback.counter += 1
        if custom_msg_callback.counter % 5 != 0:
            return
            
        # Extract a subset of points for MQTT transmission
        points_sample = []
        sample_rate = max(1, len(msg.points) // 100)  # Limit to ~100 points
        
        for i in range(0, len(msg.points), sample_rate):
            if i >= len(msg.points):
                break
                
            point = msg.points[i]
            # Convert to a simple dict format for JSON
            point_dict = {
                "x": float(point.x),
                "y": float(point.y),
                "z": float(point.z),
                "reflectivity": point.reflectivity,
                "offset_time": point.offset_time
            }
            points_sample.append(point_dict)
            
            # Hard limit at 100 points
            if len(points_sample) >= 100:
                break
        
        # Create message with header and points
        pc_data = {
            "header": {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "timebase": msg.timebase,
            "lidar_id": msg.lidar_id,
            "point_num": len(points_sample),
            "points": points_sample
        }
        
        # Publish to MQTT
        mqtt_client.publish("livox/lidar", json.dumps(pc_data))
        
    except Exception as e:
        rospy.logerr(f"Error in CustomMsg callback: {str(e)}")

# Initialize counter as an attribute of the function
custom_msg_callback.counter = 0

def check_topics():
    """Check available topics to determine which ones to subscribe to"""
    available_topics = rospy.get_published_topics()
    topic_dict = dict(available_topics)
    
    lidar_topic = None
    lidar_type = None
    
    # Check for the Livox LiDAR topic with appropriate type
    if "/livox/lidar" in topic_dict:
        lidar_topic = "/livox/lidar"
        lidar_type = topic_dict["/livox/lidar"]
        rospy.loginfo(f"Found LiDAR topic: {lidar_topic} with type: {lidar_type}")
    
    # Check for IMU topic
    imu_found = "/livox/imu" in topic_dict
    
    return lidar_topic, lidar_type, imu_found

def main():
    global mqtt_client
    
    # Initialize ROS node
    rospy.init_node('livox_mqtt_publisher', anonymous=True)
    
    # Get parameters
    mqtt_broker = rospy.get_param('~mqtt_broker', '192.168.50.191')
    mqtt_port = rospy.get_param('~mqtt_port', 1883)
    mqtt_client_id = rospy.get_param('~mqtt_client_id', f'livox_bridge_{int(time.time())}')
    
    rospy.loginfo(f"Connecting to MQTT broker at {mqtt_broker}:{mqtt_port}")
    
    # Setup MQTT client - specify older API version to avoid compatibility issues
    mqtt_client = mqtt.Client(client_id=mqtt_client_id, protocol=mqtt.MQTTv311)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    
    # Set up will message (last will and testament)
    will_msg = json.dumps({"status": "offline", "timestamp": time.time()})
    mqtt_client.will_set("livox/status", will_msg, qos=1, retain=True)
    
    # Connect to broker
    try:
        mqtt_client.connect(mqtt_broker, mqtt_port, 60)
    except Exception as e:
        rospy.logerr(f"Failed to connect to MQTT broker: {str(e)}")
        return
    
    # Start the MQTT loop in a background thread
    mqtt_client.loop_start()
    
    # Publish online status
    online_msg = json.dumps({"status": "online", "timestamp": time.time()})
    mqtt_client.publish("livox/status", online_msg, qos=1, retain=True)
    
    # Check for available topics and subscribe appropriately
    lidar_topic, lidar_type, imu_found = check_topics()
    
    if imu_found:
        rospy.loginfo("Subscribing to IMU topic: /livox/imu")
        rospy.Subscriber("/livox/imu", Imu, imu_callback, queue_size=1)
    
    if lidar_topic:
        if "CustomMsg" in lidar_type:
            rospy.loginfo("Detected Livox CustomMsg format")
            # Import dynamically to avoid dependency issues
            from livox_ros_driver2.msg import CustomMsg
            rospy.Subscriber(lidar_topic, CustomMsg, custom_msg_callback, queue_size=1)
        else:
            rospy.loginfo(f"Unknown LiDAR data format: {lidar_type}")
            rospy.loginfo("Please check ROS topic types and update the script accordingly")
    else:
        rospy.logwarn("No LiDAR topic found. Please check if the LiDAR driver is running.")
    
    rospy.loginfo("Livox MQTT publisher started")
    
    # Keep the node running
    rospy.spin()
    
    # Clean up
    online_msg = json.dumps({"status": "offline", "timestamp": time.time()})
    mqtt_client.publish("livox/status", online_msg, qos=1, retain=True)
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

if __name__ == '__main__':
    main()