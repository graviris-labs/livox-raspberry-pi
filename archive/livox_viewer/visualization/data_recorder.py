#!/usr/bin/env python3
"""
Simple Livox LiDAR data recorder that saves point cloud data to a PCD file
which can be visualized later using pcl_viewer
"""
import socket
import struct
import time
import numpy as np
import os

# Configure connection
UDP_IP = "0.0.0.0"  # Listen on all network interfaces
UDP_PORT = 65002    # Default point cloud port
OUTPUT_DIR = "/opt/livox_data"

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Function to process point cloud data
def process_packet(data):
    # Skip header information
    offset = 16  # Adjust based on your Livox packet format
    
    points = []
    while offset + 12 <= len(data):
        # Extract X, Y, Z (adjust format based on your data)
        x, y, z = struct.unpack_from('fff', data, offset)
        # Skip intensity and other fields
        points.append([x, y, z])
        offset += 16  # Move to next point (16 bytes per point)
    
    return np.array(points)

def save_to_pcd(points, filename):
    """Save points to PCD file format"""
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(points)}\n")
        f.write("DATA ascii\n")
        
        for point in points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("Livox LiDAR data recorder started")
print("Press Ctrl+C to stop recording")

try:
    frame_count = 0
    all_points = []
    start_time = time.time()
    
    while time.time() - start_time < 5:  # Record for 5 seconds
        # Receive data
        data, addr = sock.recvfrom(1500)  # UDP packet size
        
        # Process the packet
        points = process_packet(data)
        
        if len(points) > 0:
            all_points.extend(points)
            frame_count += 1
            print(f"Received frame {frame_count}, points so far: {len(all_points)}", end="\r")
    
    # Convert to numpy array
    all_points = np.array(all_points)
    
    # Save to PCD file
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"{OUTPUT_DIR}/livox_pointcloud_{timestamp}.pcd"
    save_to_pcd(all_points, filename)
    
    print(f"\nRecording complete! Saved {len(all_points)} points to {filename}")
    print(f"You can view this file using: pcl_viewer {filename}")

except KeyboardInterrupt:
    # Save on interrupt
    if len(all_points) > 0:
        all_points = np.array(all_points)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{OUTPUT_DIR}/livox_pointcloud_{timestamp}.pcd"
        save_to_pcd(all_points, filename)
        print(f"\nRecording stopped! Saved {len(all_points)} points to {filename}")
        print(f"You can view this file using: pcl_viewer {filename}")
    else:
        print("\nRecording stopped. No points recorded.")

finally:
    sock.close()