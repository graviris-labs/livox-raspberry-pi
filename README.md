# Livox 360 LiDAR on Raspberry Pi

This repository contains everything you need to run a Livox 360 LiDAR sensor with a Raspberry Pi using Docker.

## Requirements

- Raspberry Pi 4 (4GB or 8GB RAM recommended)
- Raspberry Pi OS (64-bit recommended)
- Livox 360 LiDAR sensor
- Power supply for both devices
- Ethernet connection between Pi and LiDAR

## Quick Start

1. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/livox-raspberry-pi.git
   cd livox-raspberry-pi
   ```

2. Run the setup script:
   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

3. Log out and log back in for Docker group changes to take effect.

4. Start the container:
   ```bash
   docker compose up -d
   ```

5. Check the logs:
   ```bash
   docker logs livox_lidar
   ```

## Configuration

### Network Configuration

You need to set your Raspberry Pi to have a static IP address in the same network as the Livox LiDAR. The default IP for the LiDAR is typically obtained through dynamic IP (DHCP).

If you need to set a static IP for your Raspberry Pi, edit your `/etc/dhcpcd.conf` file:

```bash
sudo nano /etc/dhcpcd.conf
```

Add the following (adjust the IP if needed):
```
interface eth0
static ip_address=192.168.1.50/24
```

### LiDAR Configuration

Edit the `config/livox_lidar_config.json` file to match your LiDAR settings. If you know your LiDAR's broadcast code, you can specify it in the configuration.

## Directory Structure

- `config/` - Contains LiDAR configuration files
- `data/` - Directory where LiDAR data can be stored
- `Dockerfile` - Instructions for building the Docker image
- `docker-compose.yml` - Docker Compose configuration file
- `setup.sh` - Setup script for installing dependencies

## Troubleshooting

### LiDAR Not Detected

1. Make sure the LiDAR is powered on
2. Check network connections
3. Verify IP configurations match
4. Check container logs: `docker logs livox_lidar`
5. Try restarting the container: `docker compose restart`

### Viewing Point Cloud Data

To visualize the point cloud data, you can:

1. Forward the data to another computer running visualization software
2. Install a visualization tool inside the Docker container
3. Use ROS tools if you're familiar with the Robot Operating System

## Advanced Usage

### Custom Build Parameters

You can modify the Dockerfile to adjust the build process according to your needs.

### Data Processing

For processing the point cloud data, you can extend this setup by:
1. Adding custom processing scripts
2. Integrating with other systems
3. Implementing SLAM or mapping algorithms

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Livox SDK: https://github.com/Livox-SDK/Livox-SDK2
- Docker for Raspberry Pi: https://www.docker.com/blog/happy-pi-day-docker-raspberry-pi/
