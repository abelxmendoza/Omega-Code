# Jetson Orin Nano — PyTorch & CUDA Setup

## Quick Reference

**Connection:** `ssh omega1@100.107.112.110`

```bash
# Setup (run on Jetson after cloning repo)
cd ~/Omega-Code
./scripts/setup_jetson_orin.sh

# Source environment
source ~/.ros2_jetson_setup.bash

# Build workspace
cd ~/omega_ws
colcon build --symlink-install
source install/setup.bash
```

**Performance mode:**
```bash
sudo jetson_clocks      # max clock speeds
sudo nvpmodel -m 0      # MAXN mode (max performance)
```

**Monitoring:**
```bash
sudo tegrastats    # system stats
nvidia-smi         # GPU stats
htop               # CPU/RAM
```

---

## Copying Files to Jetson

**Option 1 — Copy scripts directly (fastest):**
```bash
# From your laptop
ssh omega1@100.107.112.110 "mkdir -p ~/scripts"
scp scripts/install_pytorch_jetson.sh omega1@100.107.112.110:~/scripts/
scp scripts/verify_pytorch_jetson.sh omega1@100.107.112.110:~/scripts/

# On Jetson
cd ~/scripts && chmod +x *.sh && ./install_pytorch_jetson.sh
```

**Option 2 — Clone repo:**
```bash
# On Jetson
git clone <your-repo-url> ~/Omega-Code
cd ~/Omega-Code
chmod +x scripts/*.sh
./scripts/install_pytorch_jetson.sh
```

**Option 3 — rsync entire project:**
```bash
# From laptop
rsync -avz --exclude 'venv' --exclude 'node_modules' --exclude '.git' \
  ~/Desktop/Omega-Code/ omega1@100.107.112.110:~/Omega-Code/
```

---

## Installing PyTorch

### Recommended: Use the install script

```bash
# On Jetson
cd ~/Omega-Code
chmod +x scripts/install_pytorch_jetson.sh scripts/verify_pytorch_jetson.sh
./scripts/install_pytorch_jetson.sh
```

The script auto-detects your JetPack version, downloads the correct NVIDIA PyTorch wheel, installs torchvision, and verifies CUDA support. Takes 5–15 minutes.

### Manual install (if script unavailable)

**1. Check your JetPack version:**
```bash
cat /etc/nv_tegra_release
```

**2. Install from NVIDIA pre-built wheel:**
- Visit the NVIDIA PyTorch forum for the correct wheel for your JetPack version
- Example for JetPack 5.1.2:
```bash
wget https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0-cp38-cp38-linux_aarch64.whl
pip3 install torch-2.1.0-cp38-cp38-linux_aarch64.whl
pip3 install torchvision --no-deps
pip3 install pillow
```

**3. If you have a wheel file already:**
```bash
pip3 install torch-*.whl
pip3 install torchvision --no-deps
pip3 install pillow
```

### Verify installation

```bash
./scripts/verify_pytorch_jetson.sh

# Or manually:
python3 << 'EOF'
import torch
print(f"PyTorch: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    x = torch.randn(3, 3).cuda()
    print(f"GPU tensor test passed: {x.shape}")
    print(f"Device: {torch.cuda.get_device_name(0)}")
EOF
```

---

## Fix: cuDNN Library Not Found

**Error:** `libcudnn.so.8: cannot open shared object file: No such file or directory`

```bash
# Find cuDNN location
find /usr -name "libcudnn.so*" 2>/dev/null

# Add to ~/.bashrc
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=$CUDA_HOME/bin:$PATH

source ~/.bashrc
python3 -c "import torch; print(torch.cuda.is_available())"
```

---

## Troubleshooting

### `pip install torch` fails with architecture error

Standard PyPI wheels are x86_64 only. Use the Jetson-specific install script or NVIDIA pre-built wheels.

### CUDA not available after install

```bash
# Check CUDA
nvcc --version
cat /etc/nv_tegra_release

# Reinstall with correct version
pip3 uninstall torch torchvision -y
cd ~/Omega-Code && ./scripts/install_pytorch_jetson.sh

# Reboot and test
sudo reboot
python3 -c "import torch; print(torch.cuda.is_available())"
```

### torchvision build fails

```bash
pip3 install torchvision --no-deps
pip3 install pillow
```

### Out of memory / process killed

```bash
# Add swap
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Install with limited parallelism
export MAX_JOBS=1
pip3 install torch --no-cache-dir
```

### cuDNN version mismatch (wheel needs cuDNN 8, system has cuDNN 9)

This happens when using older NVIDIA wheels on newer JetPack. Find a wheel built for your exact JetPack/CUDA/cuDNN combination from the NVIDIA developer forums.

### ROS2 communication issues

```bash
# Restart daemon
ros2 daemon stop
ros2 daemon start

# Open UDP ports for DDS
sudo ufw allow 7400:7500/udp
```

### SSH issues

```bash
ssh -v omega1@100.107.112.110     # verbose connection
ssh-copy-id omega1@100.107.112.110  # copy SSH key
```

---

## System Diagnostics

```bash
#!/bin/bash
echo "=== Jetson Diagnostics ==="
echo "JetPack:"; cat /etc/nv_tegra_release 2>/dev/null || echo "Not found"
echo "CUDA:"; nvcc --version 2>/dev/null || echo "Not found"
echo "cuDNN:"; ls /usr/lib/aarch64-linux-gnu/libcudnn.so* 2>/dev/null
echo "Python:"; python3 --version
echo "PyTorch:"; python3 -c "import torch; print(f'{torch.__version__}, CUDA={torch.cuda.is_available()}')" 2>&1
echo "Disk:"; df -h /
echo "Memory:"; free -h
```

---

## Run AI Nodes (after setup)

```bash
# Vision processor
ros2 run omega_robot vision_processor

# Object detection (YOLO)
ros2 run omega_robot object_detector

# SLAM
ros2 run omega_robot slam_processor
```

See [PLATFORMS.md](PLATFORMS.md) for multi-device ROS2 setup and [ROS2.md](ROS2.md) for topic reference.
