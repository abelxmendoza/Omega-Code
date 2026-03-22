# Omega-Code Development Setup

**Cursor → GitHub → Docker (ROS2 Humble) → Pi Runtime**

The official workflow for Omega-Code: write code on your dev machine, sync via GitHub, run everything in Docker on the Pi.

## Directory Structure

```
Omega-Code/
├── .devcontainer/          # ROS2 intellisense for Cursor/VSCode
├── docker/
│   └── ros2_robot/
│       ├── Dockerfile.dev
│       ├── docker-compose.dev.yml
│       └── entrypoint.dev.sh
├── ros/
│   ├── launch/
│   │   ├── pi_only.launch.py
│   │   ├── pi_orin_hybrid.launch.py
│   │   └── omega_full.launch.py
│   └── src/
│       └── omega_robot/    # ROS2 packages
├── scripts/
│   ├── dev_workflow_sync.sh
│   └── setup_pi_docker.sh
├── servers/
│   └── robot_controller_backend/
└── ui/
    └── robot-controller-ui/
```

## Development Workflow

### Step 1: Edit Code (Dev Machine)

Edit in Cursor/VSCode:
- ROS2 packages: `ros/src/omega_robot/`
- Backend: `servers/robot_controller_backend/`
- Frontend: `ui/robot-controller-ui/`

### Step 2: Commit and Push

```bash
git add .
git commit -m "your message"
git push
```

### Step 3: Sync to Pi

**Automated (recommended):**
```bash
./scripts/dev_workflow_sync.sh [pi-hostname]
```

**Manual:**
```bash
ssh omega1-tailscale
cd ~/Omega-Code
git pull origin master
```

### Step 4: Rebuild and Run on Pi

```bash
cd ~/Omega-Code/docker/ros2_robot
docker compose -f docker-compose.dev.yml up --build
```

## Initial Setup

### Dev Machine (MacBook/Laptop)

```bash
cd ~/Desktop
git clone git@github.com:abelxmendoza/Omega-Code.git
```

Set up SSH to Pi:
```bash
ssh-keygen -t ed25519 -C "your_email@example.com"
ssh-copy-id omega1-tailscale
```

### Raspberry Pi

**Automated:**
```bash
cd ~/Omega-Code/scripts
chmod +x setup_pi_docker.sh
./setup_pi_docker.sh
```

**Manual:**
```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
# Log out and back in

# Clone repo
cd ~
git clone https://github.com/abelxmendoza/Omega-Code.git

# Build and start container
cd ~/Omega-Code/docker/ros2_robot
docker compose -f docker-compose.dev.yml build
docker compose -f docker-compose.dev.yml up -d
```

## Docker Container Details

- **Container name**: `omega_ros2`
- **Base image**: `ros:humble`
- **Volumes**: Host `~/Omega-Code` → `/workspace/Omega-Code`, `/dev` → `/dev`
- **Network**: `host` mode (required for ROS2 DDS)
- **Privileges**: `privileged` (required for hardware access)

The dev compose file runs `pi_sensor_hub.py` automatically and mounts the repo for hot-swapping — edit code on your dev machine, pull on the Pi, rebuild, and changes are live.

## Running ROS2 Nodes

```bash
# Enter container
docker exec -it omega_ros2 bash

# Inside container — source environment
source /opt/ros/humble/setup.bash
source /root/omega_ws/install/setup.bash

# Run a node
ros2 run omega_robot <node_name>

# Or use launch files
ros2 launch omega_robot <launch_file>.launch.py
```

**Useful container aliases:**
- `rebuild-ws` — rebuild ROS2 workspace
- `sync-code` — pull latest code from GitHub
- `ros-source` — source ROS2 environment
- `ros-nodes` / `ros-topics` — list nodes/topics
- `ros-echo <topic>` — echo topic messages

## Quick Reference

### Dev Machine Commands
```bash
# Sync code to Pi
./scripts/dev_workflow_sync.sh

# Run tests locally
cd ui/robot-controller-ui && npm test
cd servers/robot_controller_backend && pytest tests/
```

### Pi Commands
```bash
# Start container
~/start_omega_ros2.sh

# Sync code manually
cd ~/Omega-Code && git pull origin master

# Enter container
docker exec -it omega_ros2 bash

# View logs
docker logs omega_ros2
```

### Debugging
```bash
# Check ROS topics
docker exec -it omega_ros2 bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /omega/camera/compressed

# Rebuild workspace
cd /root/omega_ws
colcon build --symlink-install
source install/setup.bash
```

## Troubleshooting

### Container won't start
```bash
docker logs omega_ros2
docker ps -a | grep omega_ros2
# Remove and recreate:
docker rm omega_ros2
docker compose -f docker-compose.dev.yml up -d
```

### ROS2 workspace won't build
```bash
docker exec -it omega_ros2 bash
cd /root/omega_ws
colcon build --symlink-install --event-handlers console_direct+
```

### Code changes not reflected
```bash
# Ensure symlink install
docker exec omega_ros2 bash -c "cd /root/omega_ws && colcon build --symlink-install"
```

### ROS2 topics not visible across devices
```bash
# Check ROS_DOMAIN_ID matches on all machines
docker exec omega_ros2 bash -c "echo $ROS_DOMAIN_ID"
# Check CycloneDDS config
docker exec omega_ros2 bash -c "echo $CYCLONEDDS_URI"
```

## Best Practices

- Always code in Cursor/VSCode on the dev machine
- Always run ROS2 inside Docker (never install natively on Pi)
- Never modify files inside the container — use the repo volume
- Always push → pull before running
- Keep `Dockerfile.dev` lean and focused
