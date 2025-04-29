FROM ros:noetic-ros-base

RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-noetic-pcl-ros \
    ros-noetic-sensor-msgs \
    sqlite3 \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps
RUN pip3 install open3d==0.16.0

# Set up workspace and user
ENV WORKDIR=/home/rosuser
WORKDIR ${WORKDIR}
RUN useradd -ms /bin/bash rosuser
RUN mkdir -p catkin_ws/src
COPY lidar_sqlite_listener.py catkin_ws/src/lidar_sqlite_listener/

# Build workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd catkin_ws && catkin_make"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER rosuser
WORKDIR /home/rosuser

CMD ["/entrypoint.sh"]
