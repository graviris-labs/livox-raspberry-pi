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
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Clone Livox SDK2
WORKDIR /opt
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git

# Build Livox SDK2
WORKDIR /opt/Livox-SDK2
RUN mkdir build && cd build && cmake .. && make -j4 && make install

# Create directories
RUN mkdir -p /opt/livox_config
RUN mkdir -p /opt/livox_data

# Copy configuration file
COPY config/mid360_config.json /opt/livox_config/

# Set working directory
WORKDIR /opt/Livox-SDK2/build/samples/livox_lidar_quick_start

# Command to run when container starts
CMD ["./livox_lidar_quick_start", "/opt/livox_config/mid360_config.json"]
