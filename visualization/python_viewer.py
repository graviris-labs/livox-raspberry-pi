#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import socket
import struct
import time

# Configure connection
UDP_IP = "0.0.0.0"  # Listen on all network interfaces
UDP_PORT = 65002    # Default point cloud port

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Create figure for visualization
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Livox Point Cloud Visualization')

# Set axis limits for better visibility
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-2, 5])

# Function to process point cloud data
def process_packet(data):
    # Skip header information (implementation will vary based on packet format)
    offset = 16  # Adjust based on your Livox packet format
    
    points = []
    while offset + 12 <= len(data):
        # Extract X, Y, Z (adjust format based on your data)
        x, y, z = struct.unpack_from('fff', data, offset)
        points.append([x, y, z])
        offset += 16  # Move to next point (16 bytes per point)
    
    return np.array(points)

# Main visualization loop
try:
    while True:
        # Receive data
        data, addr = sock.recvfrom(1500)  # UDP packet size
        
        # Process the packet
        points = process_packet(data)
        
        if len(points) > 0:
            # Clear previous points
            ax.clear()
            
            # Plot the new points
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], 
                       c=points[:, 2], cmap='viridis', s=1)
            
            # Update plot
            plt.draw()
            plt.pause(0.01)

except KeyboardInterrupt:
    sock.close()
    plt.close()