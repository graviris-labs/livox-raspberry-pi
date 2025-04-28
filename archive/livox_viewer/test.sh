# Install dependencies
sudo apt-get update && sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-system-dev \
    libboost-thread-dev \
    libapr1-dev \
    libssl-dev \
    libcurl4-openssl-dev \
    pkg-config

# Clone Livox SDK2
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git

# Build Livox SDK2
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j4
sudo make install

# Create config directory and file
mkdir -p ~/livox_config
cat > ~/livox_config/mid360_config.json << 'EOL'
{
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_cloud_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "",
      "cmd_data_port": 56101,
      "push_msg_ip": "",
      "push_msg_port": 56201,
      "point_cloud_ip": "",
      "point_cloud_port": 56301,
      "imu_data_ip": "",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    },
    "lidar_config": [
      {
        "broadcast_code": "47MDMBV0030199",
        "enable_connect": true,
        "enable_fan": true,
        "return_mode": 0,
        "coordinate": 0,
        "imu_rate": 1,
        "extrinsic_parameter_source": 0
      }
    ]
}
EOL

# Run the sample program
cd ~/Livox-SDK2/build/samples/livox_lidar_quick_start
./livox_lidar_quick_start ~/livox_config/mid360_config.json
