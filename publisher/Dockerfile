FROM ros:noetic-ros-base AS builder

# We don't need desktop packages on a Pi
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    git build-essential cmake \
    && rm -rf /var/lib/apt/lists/*

ENV HOME=/home/builder
WORKDIR ${HOME}

# Build Livox SDK with minimal memory usage
RUN mkdir -p ${HOME}/livox-sdk && \
    cd ${HOME}/livox-sdk && \
    git clone https://github.com/EPVelasco/Livox-SDK2.git && \
    mkdir -p ./Livox-SDK2/build && \
    cd ./Livox-SDK2/build && \
    cmake .. && \
    make -j1 && \
    make install

# Final image
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    apt-utils curl wget git bash-completion build-essential sudo \
    ros-noetic-perception-pcl ros-noetic-eigen-conversions iputils-ping screen vim nano \
    python3-pip python3-numpy python3-rospy \
    ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-camera-info-manager \
    ros-noetic-compressed-image-transport \
    v4l-utils \
    && pip3 install paho-mqtt numpy opencv-python \
    && rm -rf /var/lib/apt/lists/*

# Copy the built libraries from the builder stage
COPY --from=builder /usr/local/lib/liblivox* /usr/local/lib/
COPY --from=builder /usr/local/include/livox* /usr/local/include/

# Now create the user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} livox && \
    adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} livox && \
    usermod -a -G dialout,video livox && \
    mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget && \
    cp config/99_aptget /etc/sudoers.d/99_aptget && \
    chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

ENV HOME=/home/livox
RUN mkdir -p ${HOME}/catkin_ws/src

# Build ROS driver with minimal memory usage
RUN . /opt/ros/noetic/setup.sh && \
    cd ${HOME}/catkin_ws/src && \
    git clone https://github.com/EPVelasco/livox_ros_driver2.git && \
    cd ${HOME}/catkin_ws/src/livox_ros_driver2 && \
    ./build.sh ROS1

COPY ./MID360_config.json ${HOME}/catkin_ws/src/livox_ros_driver2/config/
COPY mqtt_livox_publisher.py /home/livox/mqtt_livox_publisher.py
COPY mqtt_ir_publisher.py /home/livox/mqtt_ir_publisher.py
RUN chmod +x /home/livox/mqtt_livox_publisher.py
RUN chmod +x /home/livox/mqtt_ir_publisher.py

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["bash", "/entrypoint.sh"]
