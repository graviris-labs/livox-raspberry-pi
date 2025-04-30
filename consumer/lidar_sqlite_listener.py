#!/usr/bin/env python3
import sqlite3
import rospy
from livox_ros_driver2.msg import CustomMsg

conn = sqlite3.connect('/data/lidar_data.db')
c = conn.cursor()
c.execute('''CREATE TABLE IF NOT EXISTS lidar (
    timestamp REAL, x REAL, y REAL, z REAL, reflectivity INTEGER
)''')

def callback(msg):
    for pt in msg.points:
        c.execute("INSERT INTO lidar VALUES (?, ?, ?, ?, ?)",
                  (msg.header.stamp.to_sec(), pt.x, pt.y, pt.z, pt.reflectivity))
    conn.commit()

rospy.init_node('lidar_sqlite_listener')
rospy.Subscriber('/livox/lidar', CustomMsg, callback)
rospy.spin()
