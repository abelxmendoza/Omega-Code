# Jetson Orin Nano Quick Reference

## 🔌 Connection

```bash
# SSH to Jetson
ssh omega1@100.107.112.110

# Test connectivity
ping 100.107.112.110
```

## 🚀 Quick Setup

```bash
# 1. On Jetson
cd ~/Omega-Code
./scripts/setup_jetson_orin.sh

# 2. Source environment
source ~/.ros2_jetson_setup.bash

# 3. Build workspace
cd ~/omega_ws
colcon build --symlink-install
source install/setup.bash
```

## 🎯 Common Commands

### ROS2 Basics
```bash
# List topics
ros2 topic list

# Echo topic
ros2 topic echo /omega/telemetry

# List nodes
ros2 node list

# Check node info
ros2 node info /vision_processor
```

### Run Vision Node
```bash
# On Jetson
ros2 run omega_robot vision_processor
```

### Performance Mode
```bash
# Enable max performance
sudo jetson_clocks
sudo nvpmodel -m 0  # MAXN mode
```

### System Monitoring
```bash
# System stats
sudo tegrastats

# GPU stats
nvidia-smi

# CPU/Memory
htop
```

## 📡 Multi-Device Testing

### Terminal 1: Laptop
```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/docker/ros2_robot/config/cyclonedds.xml
ros2 run demo_nodes_cpp talker
```

### Terminal 2: Jetson
```bash
ssh omega1@100.107.112.110
source ~/.ros2_jetson_setup.bash
ros2 run demo_nodes_cpp listener
```

## 🔧 Troubleshooting

### SSH Issues
```bash
# Test connection
ssh -v omega1@100.107.112.110

# Copy SSH key
ssh-copy-id omega1@100.107.112.110
```

### ROS2 Communication Issues
```bash
# Restart daemon
ros2 daemon stop
ros2 daemon start

# Check firewall
sudo ufw allow 7400:7500/udp
```

### CUDA/PyTorch Issues
```bash
# Check CUDA
nvcc --version

# Verify PyTorch installation
./scripts/verify_pytorch_jetson.sh

# Reinstall PyTorch if needed
./scripts/install_pytorch_jetson.sh

# Test PyTorch CUDA
python3 -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"
```

## 📚 Documentation

- Full Guide: [JETSON_ORIN_INTEGRATION.md](JETSON_ORIN_INTEGRATION.md)
- Multi-Device: [ROS2_MULTIDEVICE_SETUP.md](ROS2_MULTIDEVICE_SETUP.md)
- Architecture: [ROS2_ARCHITECTURE.md](ROS2_ARCHITECTURE.md)

