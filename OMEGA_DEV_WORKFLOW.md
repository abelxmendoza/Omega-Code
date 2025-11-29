# Omega Development Workflow

## Overview

**Goal**: Create a clean, professional robotics workflow where:
- **MacBook** is the development machine (no ROS required)
- **GitHub** is the source of truth
- **Raspberry Pi** runs ROS2 inside Docker container
- **Jetson Orin Nano** integrates seamlessly (future)

**Benefits**:
- ✅ No ROS required on MacBook
- ✅ Fast local development using Cursor + VSCode
- ✅ Pi stays clean, uses ROS2 inside Docker
- ✅ GitHub keeps version history and CI
- ✅ Hot-swapping code via git pull in Docker container
- ✅ Future-proof for Orin Nano AI brain

## Directory Structure

```
Omega-Code/
├── ros/
│   └── src/
│       └── omega_robot/          # ROS2 packages
├── docker/
│   └── ros2_robot/
│       ├── Dockerfile.dev        # Development container
│       ├── docker-compose.dev.yml
│       └── entrypoint.dev.sh
├── scripts/
│   ├── dev_workflow_sync.sh      # MacBook → GitHub → Pi sync
│   └── setup_pi_docker.sh        # Initial Pi setup
├── servers/
│   └── robot_controller_backend/ # Backend services
└── ui/
    └── robot-controller-ui/      # Frontend
```

## Docker Setup

### Container Configuration

- **Container Name**: `omega_ros2`
- **Base Image**: `ros:humble`
- **Volumes**:
  - Host: `/home/omega1/Omega-Code` → Container: `/workspace/Omega-Code`
  - Host: `/dev` → Container: `/dev` (for hardware access)
- **Network**: `host` mode (for ROS2 communication)
- **Privileges**: `privileged` (for hardware access)

### Docker Run Command

```bash
cd ~/Omega-Code/docker/ros2_robot
docker compose -f docker-compose.dev.yml up -d
```

Or use the helper script:
```bash
~/start_omega_ros2.sh
```

## Development Workflow

### Step 1: Edit Code on MacBook (Cursor)

1. **Clone repository** (if not already):
   ```bash
   cd ~/Desktop
   git clone git@github.com:abelxmendoza/Omega-Code.git
   cd Omega-Code
   ```

2. **Edit code** using Cursor/VSCode:
   - Edit ROS2 packages in `ros/src/omega_robot/`
   - Edit backend in `servers/robot_controller_backend/`
   - Edit frontend in `ui/robot-controller-ui/`

3. **Test locally** (if applicable):
   ```bash
   # Frontend tests
   cd ui/robot-controller-ui && npm test
   
   # Backend tests
   cd servers/robot_controller_backend && pytest tests/
   ```

4. **Commit and push**:
   ```bash
   git add -A
   git commit -m "Your commit message"
   git push origin master
   ```

### Step 2: Sync to Pi

**Option A: Automated Sync Script** (Recommended)

From MacBook:
```bash
./scripts/dev_workflow_sync.sh [pi-hostname]
```

This script:
1. Commits and pushes to GitHub (if uncommitted changes)
2. Pulls latest code on Pi
3. Rebuilds ROS2 workspace in Docker container

**Option B: Manual Sync**

1. **SSH to Pi**:
   ```bash
   ssh omega1-tailscale  # or your Pi hostname
   ```

2. **Pull code**:
   ```bash
   cd ~/Omega-Code
   git pull origin master
   ```

3. **Rebuild ROS2 workspace**:
   ```bash
   docker exec -it omega_ros2 bash
   cd /root/omega_ws
   colcon build --symlink-install
   source install/setup.bash
   exit
   ```

Or use the sync script on Pi:
```bash
~/sync_omega_code.sh
```

### Step 3: Run ROS2 Nodes

**Enter Docker container**:
```bash
docker exec -it omega_ros2 bash
```

**Inside container**:
```bash
# Source ROS2 environment (if not already sourced)
source /opt/ros/humble/setup.bash
source /root/omega_ws/install/setup.bash

# List available nodes
ros2 run omega_robot --help

# Run a node
ros2 run omega_robot <node_name>

# Or use launch files
ros2 launch omega_robot <launch_file>.launch.py
```

**Useful aliases** (already set up in container):
- `rebuild-ws` - Rebuild ROS2 workspace
- `sync-code` - Pull latest code from GitHub
- `ros-source` - Source ROS2 environment
- `ros-nodes` - List ROS2 nodes
- `ros-topics` - List ROS2 topics
- `ros-echo <topic>` - Echo topic messages

## Initial Setup

### On MacBook

1. **Clone repository**:
   ```bash
   cd ~/Desktop
   git clone git@github.com:abelxmendoza/Omega-Code.git
   ```

2. **Set up SSH keys** (if not already):
   ```bash
   ssh-keygen -t ed25519 -C "your_email@example.com"
   ssh-copy-id omega1-tailscale  # or your Pi hostname
   ```

### On Raspberry Pi

**Option A: Automated Setup** (Recommended)

Run the setup script:
```bash
curl -fsSL https://raw.githubusercontent.com/abelxmendoza/Omega-Code/master/scripts/setup_pi_docker.sh | bash
```

Or manually:
```bash
cd ~
git clone https://github.com/abelxmendoza/Omega-Code.git
cd Omega-Code/scripts
chmod +x setup_pi_docker.sh
./setup_pi_docker.sh
```

**Option B: Manual Setup**

1. **Install Docker**:
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   sudo usermod -aG docker $USER
   # Log out and back in
   ```

2. **Clone repository**:
   ```bash
   cd ~
   git clone https://github.com/abelxmendoza/Omega-Code.git
   ```

3. **Build Docker image**:
   ```bash
   cd ~/Omega-Code/docker/ros2_robot
   docker compose -f docker-compose.dev.yml build
   ```

4. **Start container**:
   ```bash
   docker compose -f docker-compose.dev.yml up -d
   ```

## Hot-Swapping Code

After making changes on MacBook and pushing to GitHub:

**Quick sync** (from MacBook):
```bash
./scripts/dev_workflow_sync.sh
```

**Or manually** (from Pi):
```bash
cd ~/Omega-Code
git pull origin master
docker exec omega_ros2 bash -c "cd /root/omega_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash"
```

The workspace uses `--symlink-install`, so code changes are immediately reflected after rebuild.

## Troubleshooting

### Container won't start

```bash
# Check Docker logs
docker logs omega_ros2

# Check if container exists
docker ps -a | grep omega_ros2

# Remove and recreate
docker rm omega_ros2
cd ~/Omega-Code/docker/ros2_robot
docker compose -f docker-compose.dev.yml up -d
```

### ROS2 workspace won't build

```bash
# Enter container
docker exec -it omega_ros2 bash

# Check workspace
cd /root/omega_ws
ls -la src/

# Rebuild with verbose output
colcon build --symlink-install --event-handlers console_direct+
```

### Code changes not reflected

```bash
# Ensure symlink install
docker exec omega_ros2 bash -c "cd /root/omega_ws && colcon build --symlink-install"

# Check if package is linked correctly
docker exec omega_ros2 bash -c "ls -la /root/omega_ws/src/omega_robot"
```

### Network issues (ROS2 topics not visible)

```bash
# Check ROS_DOMAIN_ID matches
docker exec omega_ros2 bash -c "echo \$ROS_DOMAIN_ID"

# Check CycloneDDS config
docker exec omega_ros2 bash -c "echo \$CYCLONEDDS_URI"

# List topics
docker exec omega_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
```

## Future: Jetson Orin Nano Integration

When adding Jetson Orin Nano:

1. **Orin runs AI brain** (inference, navigation)
2. **Pi runs sensor hub** (camera, sensors)
3. **Both communicate via ROS2** (same domain ID)
4. **MacBook remains dev machine** (no ROS needed)

The Docker setup on Pi remains the same. Orin will run its own ROS2 nodes (either natively or in Docker).

## Quick Reference

### MacBook Commands

```bash
# Sync code to Pi
./scripts/dev_workflow_sync.sh

# Run tests
cd ui/robot-controller-ui && npm test
cd servers/robot_controller_backend && pytest tests/
```

### Pi Commands

```bash
# Start container
~/start_omega_ros2.sh

# Sync code
~/sync_omega_code.sh

# Enter container
docker exec -it omega_ros2 bash

# View logs
docker logs omega_ros2
```

### Container Commands

```bash
# Rebuild workspace
rebuild-ws

# Sync code
sync-code

# List nodes
ros-nodes

# List topics
ros-topics

# Echo topic
ros-echo /omega/camera/compressed
```

## CI/CD Integration

The GitHub Actions workflows automatically:
- ✅ Run tests on push/PR
- ✅ Build Docker images
- ✅ Deploy to Pi (if configured)

See `.github/workflows/ci.yml` for details.

