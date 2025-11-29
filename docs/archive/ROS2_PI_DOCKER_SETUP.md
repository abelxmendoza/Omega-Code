# ROS2 Pi Docker Setup Guide

This guide explains how to use ROS2 with Docker on Raspberry Pi OS, and how the laptop (native ROS2) communicates with Pi (Docker ROS2).

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Multi-Device ROS2                     │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────┐         ┌──────────────┐            │
│  │   Laptop     │         │  Raspberry   │            │
│  │  (Ubuntu)    │         │     Pi 4B    │            │
│  │              │         │              │            │
│  │ Native ROS2  │◄───────►│ Docker ROS2  │            │
│  │   Rolling    │  DDS    │   Humble     │            │
│  │              │ Network │              │            │
│  │ • rclpy      │         │ • Containers │            │
│  │ • Direct     │         │ • Pi OS      │            │
│  └──────────────┘         └──────────────┘            │
│                                                         │
│              ROS2 DDS (CycloneDDS)                      │
│              Domain ID: 0                               │
└─────────────────────────────────────────────────────────┘
```

## Pi Setup (Raspberry Pi OS)

### 1. Install Docker

```bash
# On Pi
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker pi
```

### 2. Build ROS2 Docker Image

```bash
# On Pi
cd ~/Omega-Code/docker/ros2_robot
sudo docker build -t omega_robot:latest -f Dockerfile ../..
```

### 3. Configure CycloneDDS

Update `docker/ros2_robot/config/cyclonedds.xml` on Pi:

```xml
<CycloneDDS>
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.100"/>  <!-- Laptop IP -->
        <Peer address="192.168.1.107"/>  <!-- Pi IP -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### 4. Start ROS2 Containers on Pi

```bash
# On Pi
cd ~/Omega-Code/docker/ros2_robot
sudo docker-compose up -d
```

## Laptop Setup (Ubuntu)

### 1. Native ROS2 Rolling

```bash
# On Laptop
source ~/.ros2_rolling_setup.bash
source ~/omega_ws/install/setup.bash
```

### 2. Configure CycloneDDS

Update `docker/ros2_robot/config/cyclonedds.xml` on laptop:

```xml
<CycloneDDS>
  <Domain id="any">
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.100"/>  <!-- Laptop IP -->
        <Peer address="192.168.1.107"/>  <!-- Pi IP -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

### 3. Set Environment Variables

```bash
# On Laptop - for backend to control Pi Docker
export PI_SSH_HOST=pi@192.168.1.107
export PI_SSH_KEY=~/.ssh/id_rsa  # Optional, if using key auth
export PI_DOCKER_COMPOSE_PATH=/home/pi/Omega-Code/docker/ros2_robot/docker-compose.yml
export ROS_NATIVE_MODE=true  # Use native ROS2 on laptop
```

## Backend Configuration

The backend can run on either laptop or Pi:

### Running on Laptop (Control Pi Docker)

```bash
# On Laptop
cd servers/robot_controller_backend

# Set environment
export PI_SSH_HOST=pi@192.168.1.107
export ROS_NATIVE_MODE=true

# Start backend
source venv/bin/activate
python main_api.py
```

The backend will:
- Use native ROS2 on laptop for local topics
- Control Pi Docker containers via SSH
- Bridge between laptop native ROS2 and Pi Docker ROS2

### Running on Pi (Local Docker)

```bash
# On Pi
cd ~/Omega-Code/servers/robot_controller_backend

# Don't set PI_SSH_HOST (will use local Docker)
# Don't set ROS_NATIVE_MODE (Pi uses Docker)

# Start backend
source venv/bin/activate
python main_api.py
```

## Communication Flow

### Laptop → Pi

1. **Laptop publishes** to ROS2 topic (native)
2. **CycloneDDS** discovers Pi via peer configuration
3. **Pi Docker container** receives message
4. **Pi nodes** process and respond

### Pi → Laptop

1. **Pi Docker container** publishes to ROS2 topic
2. **CycloneDDS** discovers laptop via peer configuration
3. **Laptop native ROS2** receives message
4. **Backend/UI** processes message

## API Usage

### Check Status (from Laptop)

```bash
# Backend will check Pi Docker containers via SSH
curl http://localhost:8000/api/ros/status
```

Response:
```json
{
  "containers": [...],
  "topics": [...],
  "mode": "docker_pi",
  "location": "pi"
}
```

### Control Pi Containers (from Laptop)

```bash
# Start containers on Pi
curl -X POST http://localhost:8000/api/ros/control \
  -H "Content-Type: application/json" \
  -d '{"action": "start"}'
```

### List Topics (from Laptop)

```bash
# Will show topics from both laptop (native) and Pi (Docker)
curl http://localhost:8000/api/ros/topics
```

## Testing Communication

### On Laptop

```bash
# Terminal 1: Listen to topics
source ~/.ros2_rolling_setup.bash
ros2 topic echo /omega/telemetry
```

### On Pi

```bash
# Check if containers are running
sudo docker ps

# Check topics in container
sudo docker exec -it omega_robot_telemetry_publisher_1 \
  bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

### Verify Communication

```bash
# On Pi: Publish test message
sudo docker exec -it omega_robot_telemetry_publisher_1 \
  bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub /test std_msgs/String 'data: hello'"

# On Laptop: Should receive message
ros2 topic echo /test
```

## Troubleshooting

### Pi Containers Not Found

```bash
# Check SSH connection
ssh pi@192.168.1.107 "docker ps"

# Verify PI_SSH_HOST is set
echo $PI_SSH_HOST
```

### Topics Not Visible

1. **Check CycloneDDS config** on both devices
2. **Verify IPs** in peer configuration
3. **Check firewall** (UDP ports 7400-7500)
4. **Verify ROS_DOMAIN_ID** matches (default: 0)

### Docker Permission Issues

```bash
# On Pi
sudo usermod -aG docker pi
# Logout and login again
```

### Network Issues

```bash
# Test connectivity
ping 192.168.1.107  # From laptop to Pi
ping 192.168.1.100  # From Pi to laptop

# Check ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

## Best Practices

1. **Use native ROS2 on laptop** for development (faster)
2. **Use Docker on Pi** for consistency (Raspberry Pi OS)
3. **Configure CycloneDDS peers** on both devices
4. **Use SSH keys** for Pi access (not passwords)
5. **Monitor network** for DDS discovery issues

## Environment Variables Summary

**Laptop:**
```bash
ROS_NATIVE_MODE=true
PI_SSH_HOST=pi@192.168.1.107
PI_SSH_KEY=~/.ssh/id_rsa
PI_DOCKER_COMPOSE_PATH=/home/pi/Omega-Code/docker/ros2_robot/docker-compose.yml
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file://$HOME/Omega-Code/docker/ros2_robot/config/cyclonedds.xml
```

**Pi:**
```bash
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///root/omega_ws/config/cyclonedds.xml
```

---

**Last Updated**: 2024  
**ROS2 Versions**: Rolling (Laptop), Humble (Pi Docker)  
**OS**: Ubuntu 24.04 (Laptop), Raspberry Pi OS (Pi)

