#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("camera_publisher")
    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
    bridge = CvBridge()
    
    # For Ubuntu with IMX219 camera, it should be video0
    cap = cv2.VideoCapture(0)  # adjust index if needed

    if not cap.isOpened():
        rospy.logerr("Failed to open IR camera.")
        return

    # Try to set camera properties
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 10)

    rospy.loginfo("Camera opened successfully, publishing to /camera/image_raw")
    rate = rospy.Rate(10)  # 10 FPS

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame.")
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()
