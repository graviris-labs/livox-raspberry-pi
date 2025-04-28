FROM arm64v8/debian:bullseye-slim

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-system-dev \
    libboost-thread-dev \
    libapr1-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Clone Livox SDK2
WORKDIR /opt
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git

# Build Livox SDK2
WORKDIR /opt/Livox-SDK2
RUN mkdir build && cd build && cmake .. && make -j4 && make install

# Create a directory for configs
RUN mkdir -p /opt/livox_config

# Create a directory for output data
RUN mkdir -p /opt/livox_data

# Instead of copying from a non-existent location, we'll create the config file in a later step

# Set working directory
WORKDIR /opt/Livox-SDK2/build/samples/livox_lidar_quick_start

# Command to run when container starts
CMD ["./livox_lidar_quick_start", "/opt/livox_config/livox_lidar_config.json"]