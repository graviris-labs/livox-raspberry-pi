#!/usr/bin/env python3

import rospy
import numpy as np
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def calculate_pseudo_ndvi(image):
    """
    Calculate pseudo-NDVI from a NoIR camera without additional filters
    Using the differential sensitivity of the sensor channels to NIR
    """
    # Split the BGR channels
    b, g, r = cv2.split(image)
    
    # Red channel has more NIR sensitivity, blue has less
    # Use this difference for a pseudo-NDVI
    r_float = r.astype(float)
    b_float = b.astype(float)
    
    # Avoid division by zero
    denominator = r_float + b_float
    denominator[denominator == 0] = 1
    
    # Calculate pseudo-NDVI
    pseudo_ndvi = (r_float - b_float) / denominator
    
    # Scale to 0-255 for visualization
    ndvi_scaled = ((pseudo_ndvi + 1) / 2 * 255).astype(np.uint8)
    
    # Create a color map for better visualization
    ndvi_color = cv2.applyColorMap(ndvi_scaled, cv2.COLORMAP_JET)
    
    return ndvi_color, pseudo_ndvi

def main():
    rospy.init_node("camera_publisher")
    pub_raw = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
    pub_ndvi = rospy.Publisher("/camera/ndvi", Image, queue_size=1)
    bridge = CvBridge()
    
    rospy.loginfo("Starting NoIR camera publisher with OpenCV...")
    
    # Try different camera indices
    for cam_idx in range(3):
        cap = cv2.VideoCapture(cam_idx)
        if cap.isOpened():
            rospy.loginfo(f"Camera opened at index {cam_idx}")
            break
        else:
            rospy.logwarn(f"Failed to open camera at index {cam_idx}")
    
    if not cap.isOpened():
        rospy.logerr("Could not open any camera")
        return
    
    # Set properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Get actual properties
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
    rospy.loginfo(f"Camera opened with resolution: {actual_width}x{actual_height}")
    
    # Skip first few frames
    for _ in range(3):
        cap.read()
    
    rate = rospy.Rate(10)  # 10 FPS
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame.")
            rate.sleep()
            continue
        
        try:
            # Calculate pseudo-NDVI
            ndvi_color, ndvi_raw = calculate_pseudo_ndvi(frame)
            
            # Create ROS messages
            msg_raw = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg_raw.header.stamp = rospy.Time.now()
            pub_raw.publish(msg_raw)
            
            msg_ndvi = bridge.cv2_to_imgmsg(ndvi_color, encoding="bgr8")
            msg_ndvi.header.stamp = rospy.Time.now()
            pub_ndvi.publish(msg_ndvi)
            
            # Log occasionally
            if int(rospy.get_time()) % 10 == 0:
                rospy.loginfo("Camera publishing successfully")
                
        except Exception as e:
            rospy.logerr(f"Error processing frame: {e}")
        
        rate.sleep()
    
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Camera publisher failed: {e}")
