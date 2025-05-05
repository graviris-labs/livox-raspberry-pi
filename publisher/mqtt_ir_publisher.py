#!/usr/bin/env python3

import rospy
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

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
    
    rospy.loginfo("Starting NoIR camera publisher with picamera2...")
    
    try:
        # Import and initialize picamera2
        from picamera2 import Picamera2
        picam2 = Picamera2()
        
        # Configure camera for preview mode
        config = picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
        picam2.configure(config)
        
        # Start camera
        picam2.start()
        rospy.loginfo("NoIR Camera started successfully")
        
        # Wait for camera to initialize
        time.sleep(2)
        
        rate = rospy.Rate(10)  # 10 FPS
        
        while not rospy.is_shutdown():
            try:
                # Capture frame
                frame = picam2.capture_array()
                
                # Convert from RGB to BGR for OpenCV processing
                if frame.shape[2] == 3:  # If it's RGB
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                else:
                    frame_bgr = frame
                
                # Calculate pseudo-NDVI
                ndvi_color, ndvi_raw = calculate_pseudo_ndvi(frame_bgr)
                
                # Create ROS messages
                msg_raw = bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
                msg_raw.header.stamp = rospy.Time.now()
                pub_raw.publish(msg_raw)
                
                msg_ndvi = bridge.cv2_to_imgmsg(ndvi_color, encoding="bgr8")
                msg_ndvi.header.stamp = rospy.Time.now()
                pub_ndvi.publish(msg_ndvi)
                
                # Log occasionally
                if int(rospy.get_time()) % 10 == 0:
                    rospy.loginfo("NoIR Camera publishing raw and pseudo-NDVI images")
                
            except Exception as e:
                rospy.logerr(f"Error capturing frame: {e}")
            
            rate.sleep()
        
        # Clean up
        picam2.close()
        
    except ImportError as e:
        rospy.logerr(f"Failed to import picamera2: {e}")
    except Exception as e:
        rospy.logerr(f"Camera initialization failed: {e}")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr(f"Camera publisher failed: {e}")
