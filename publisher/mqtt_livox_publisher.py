#!/usr/bin/env python3

import rospy
import json
import numpy as np
import paho.mqtt.client as mqtt
from sensor_msgs.msg import PointCloud2, Imu
import struct
import time

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

def extract_xyz(cloud_msg):
    """Extract XYZ points from a PointCloud2 message"""
    # Get the field offsets for x, y, z
    x_offset = y_offset = z_offset = None
    for field in cloud_msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset
    
    if x_offset is None or y_offset is None or z_offset is None:
        rospy.logerr("Point cloud does not have x, y, z fields")
        return []
    
    # Extract points (downsampled)
    points = []
    point_step = cloud_msg.point_step
    
    # Use a fixed stride to downsample (e.g., every 50th point)
    stride = 50
    
    for i in range(0, cloud_msg.width * cloud_msg.height, stride):
        if i * point_step >= len(cloud_msg.data):
            break
            
        # Extract x, y, z
        offset = i * point_step
        x = struct.unpack_from('f', cloud_msg.data, offset + x_offset)[0]
        y = struct.unpack_from('f', cloud_msg.data, offset + y_offset)[0]
        z = struct.unpack_from('f', cloud_msg.data, offset + z_offset)[0]
        
        # Only include valid points
        if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
            points.append({"x": float(x), "y": float(y), "z": float(z)})
            
        # Limit to 100 points maximum to keep message size reasonable
        if len(points) >= 100:
            break
            
    return points

def pointcloud_callback(msg):
    """Callback for PointCloud2 messages"""
    try:
        # Only process every 5th message to reduce network load
        pointcloud_callback.counter += 1
        if pointcloud_callback.counter % 5 != 0:
            return
            
        # Extract points
        points = extract_xyz(msg)
        
        # Create message with header and points
        pc_data = {
            "header": {
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id
            },
            "points": points
        }
        
        # Publish to MQTT
        mqtt_client.publish("livox/lidar", json.dumps(pc_data))
        
    except Exception as e:
        rospy.logerr(f"Error in PointCloud callback: {str(e)}")

# Initialize counter as an attribute of the function
pointcloud_callback.counter = 0

def main():
    global mqtt_client
    
    # Initialize ROS node
    rospy.init_node('livox_mqtt_publisher', anonymous=True)
    
    # Get parameters
    mqtt_broker = rospy.get_param('~mqtt_broker', '192.168.50.191')
    mqtt_port = rospy.get_param('~mqtt_port', 1883)
    mqtt_client_id = rospy.get_param('~mqtt_client_id', 'livox_bridge_' + str(int(time.time())))
    
    rospy.loginfo(f"Connecting to MQTT broker at {mqtt_broker}:{mqtt_port}")
    
    # Setup MQTT client
    mqtt_client = mqtt.Client(mqtt_client_id)
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
    
    # Subscribe to Livox topics
    rospy.Subscriber("/livox/imu", Imu, imu_callback, queue_size=1)
    rospy.Subscriber("/livox/lidar", PointCloud2, pointcloud_callback, queue_size=1)
    
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