FROM ros:noetic-ros-base AS builder

# Install minimal build deps for Livox SDK
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    git build-essential cmake \
    && rm -rf /var/lib/apt/lists/*

ENV HOME=/home/builder
WORKDIR ${HOME}

# Build Livox SDK
RUN mkdir -p livox-sdk && \
    cd livox-sdk && \
    git clone https://github.com/EPVelasco/Livox-SDK2.git && \
    mkdir -p Livox-SDK2/build && \
    cd Livox-SDK2/build && \
    cmake .. && \
    make -j1 && \
    make install

# Final stage
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    sqlite3 \
    python3-pip \
    apt-utils curl wget git bash-completion build-essential sudo \
    ros-noetic-perception-pcl ros-noetic-eigen-conversions iputils-ping screen vim nano \
    && rm -rf /var/lib/apt/lists/*

# Install runtime deps + ROS tools + SQLite
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    sqlite3 \
    git \
    sudo \
    bash-completion \
    python3-pip \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy the built libraries from the builder stage
COPY --from=builder /usr/local/lib/liblivox* /usr/local/lib/
COPY --from=builder /usr/local/include/livox* /usr/local/include/

# Now create the user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} livox && \
    adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} livox && \
    usermod -a -G dialout livox && \
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

COPY lidar_sqlite_listener.py ${HOME}/lidar_sqlite_listener.py

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

CMD ["bash", "/entrypoint.sh"]
