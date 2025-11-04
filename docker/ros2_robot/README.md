# ROS 2 Docker Setup for Omega Robot

This directory contains Docker configuration for running ROS 2 Humble nodes in a containerized environment on Raspberry Pi OS.

## Overview

The Docker setup includes:
- **Dockerfile**: Builds ROS 2 Humble image with CycloneDDS support
- **entrypoint.sh**: Sets up ROS 2 environment and sources workspace
- **docker-compose.yml**: Orchestrates telemetry publisher and listener nodes
- **config/cyclonedds.xml**: CycloneDDS configuration for peer discovery

## Quick Start

### Build

```bash
cd docker/ros2_robot
sudo docker build -t omega_robot:latest -f Dockerfile ../..
```

### Run with Docker Compose

```bash
cd docker/ros2_robot
sudo docker-compose up -d
```

### Run Individual Nodes

```bash
# Publisher
sudo docker run -it --rm --network host \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  omega_robot:latest \
  ros2 run omega_robot telemetry_publisher

# Listener (in another terminal)
sudo docker run -it --rm --network host \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  omega_robot:latest \
  ros2 run omega_robot telemetry_listener
```

## Testing

### Verify Topics

```bash
# Get container ID
sudo docker ps | grep omega_robot

# Enter container
sudo docker exec -it <container_id> bash

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
source /root/omega_ws/install/setup.bash

# List topics
ros2 topic list

# Echo telemetry
ros2 topic echo /omega/telemetry
```

### Test Networking

```bash
# From container, ping host
ping -c 3 <host_ip>

# Check CycloneDDS discovery
export CYCLONEDDS_URI=file:///root/omega_ws/config/cyclonedds.xml
ros2 topic list
```

## Configuration

### CycloneDDS Peer Discovery

Edit `config/cyclonedds.xml` to add your network peers:

```xml
<Peers>
  <Peer address="192.168.1.107"/> <!-- Raspberry Pi -->
  <Peer address="192.168.1.202"/> <!-- MacBook -->
</Peers>
```

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROS_DOMAIN_ID` | `0` | ROS 2 domain ID for topic isolation |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | ROS 2 middleware implementation |
| `CYCLONEDDS_URI` | `file:///root/omega_ws/config/cyclonedds.xml` | CycloneDDS config file path |

### Development Mode

To mount local workspace for live editing, add to `docker-compose.yml`:

```yaml
services:
  telemetry_publisher:
    volumes:
      - ../../ros/src:/root/omega_ws/src
```

Then rebuild workspace inside container:

```bash
sudo docker exec -it <container_id> bash
source /opt/ros/humble/setup.bash
cd /root/omega_ws
colcon build --symlink-install
source install/setup.bash
```

## Package Structure

The `omega_robot` package is located at `ros/src/omega_robot/`:

```
ros/src/omega_robot/
├── omega_robot/
│   ├── __init__.py
│   ├── telemetry_publisher.py
│   └── telemetry_listener.py
├── package.xml
├── setup.py
└── resource/
    └── omega_robot
```

## Troubleshooting

### Container won't start

- Check Docker logs: `sudo docker logs <container_id>`
- Verify ROS 2 workspace built successfully: `sudo docker build -t omega_robot:latest -f Dockerfile ../..`
- Ensure entrypoint script is executable: `chmod +x entrypoint.sh`

### Topics not visible

- Verify `ROS_DOMAIN_ID` matches across containers
- Check CycloneDDS configuration XML syntax
- Ensure `network_mode: host` is set for host network access
- Check firewall rules on Raspberry Pi

### Build errors

- Ensure `ros/src/omega_robot/` package structure is correct
- Verify `package.xml` and `setup.py` are valid ROS 2 format
- Check Python dependencies in `setup.py`

## Architecture

- **Base Image**: `ros:humble` (official ROS 2 Humble Docker image)
- **Middleware**: CycloneDDS (`ros-humble-rmw-cyclonedds-cpp`)
- **Workspace**: `/root/omega_ws` inside container
- **Network**: `host` mode for direct host access
- **Build System**: `colcon` with `--symlink-install` for development

## Integration with Omega-Code

This Docker setup integrates with the main Omega-Code repository:

- ROS 2 packages live in `ros/src/omega_robot/`
- Docker config lives in `docker/ros2_robot/`
- Main README.md includes ROS 2 Docker quick start
- Compatible with Raspberry Pi OS (Debian-based, apt packages)

For cross-platform development:
- Develop on MacBook (macOS)
- Deploy to Raspberry Pi OS (64-bit)
- Use Docker for consistent ROS 2 environment

