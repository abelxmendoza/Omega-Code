# Omega Dev Workflow - Quick Start

## 🚀 5-Minute Setup

### MacBook Setup

```bash
# 1. Clone repo
cd ~/Desktop
git clone git@github.com:abelxmendoza/Omega-Code.git
cd Omega-Code

# 2. Set up SSH to Pi (if not already)
ssh-copy-id omega1-tailscale  # or your Pi hostname
```

### Pi Setup

```bash
# SSH to Pi
ssh omega1-tailscale

# Run setup script
cd ~
git clone https://github.com/abelxmendoza/Omega-Code.git
cd Omega-Code/scripts
chmod +x setup_pi_docker.sh
./setup_pi_docker.sh
```

## 🔄 Daily Workflow

### 1. Edit Code (MacBook)
```bash
cd ~/Desktop/Omega-Code
# Edit files in Cursor/VSCode
```

### 2. Sync to Pi
```bash
# From MacBook
./scripts/dev_workflow_sync.sh
```

### 3. Test on Pi
```bash
# SSH to Pi
ssh omega1-tailscale

# Enter Docker container
docker exec -it omega_ros2 bash

# Run ROS2 nodes
ros2 run omega_robot <node_name>
```

## 📋 Common Commands

### MacBook
- `./scripts/dev_workflow_sync.sh` - Sync code to Pi
- `git push` - Push to GitHub

### Pi
- `~/start_omega_ros2.sh` - Start container
- `~/sync_omega_code.sh` - Sync code and rebuild
- `docker exec -it omega_ros2 bash` - Enter container

### Inside Container
- `rebuild-ws` - Rebuild ROS2 workspace
- `sync-code` - Pull latest code
- `ros-nodes` - List nodes
- `ros-topics` - List topics

## 🆘 Troubleshooting

**Container won't start?**
```bash
docker logs omega_ros2
docker rm omega_ros2
cd ~/Omega-Code/docker/ros2_robot
docker compose -f docker-compose.dev.yml up -d
```

**Code changes not reflected?**
```bash
docker exec omega_ros2 rebuild-ws
```

**Need help?**
See `OMEGA_DEV_WORKFLOW.md` for detailed documentation.

