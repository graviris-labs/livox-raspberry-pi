#!/usr/bin/env python3

import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("camera_publisher")
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
    bridge = CvBridge()
    
    # Try to open camera with default settings first
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Failed to open camera with default settings")
        return
    
    # Set properties - let OpenCV negotiate with the driver
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
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Create ROS image message
            msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
            msg.header.stamp = rospy.Time.now()
            pub.publish(msg)
            
            # Log occasionally
            if rospy.get_time() % 10 < 0.1:
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
