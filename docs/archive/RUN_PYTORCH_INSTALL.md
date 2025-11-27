# Running PyTorch Installation on Jetson

## Quick Start

### Option 1: Run via SSH (Recommended)

```bash
# 1. SSH to your Jetson
ssh omega1@100.107.112.110

# 2. Navigate to Omega-Code (or copy scripts if needed)
cd ~/Omega-Code

# If scripts aren't there, copy them:
# From your laptop:
scp scripts/install_pytorch_jetson.sh scripts/verify_pytorch_jetson.sh omega1@100.107.112.110:~/Omega-Code/scripts/

# 3. Make scripts executable
chmod +x scripts/install_pytorch_jetson.sh scripts/verify_pytorch_jetson.sh

# 4. Run the installer
./scripts/install_pytorch_jetson.sh
```

### Option 2: Run Directly on Jetson

If you're already logged into the Jetson:

```bash
cd ~/Omega-Code
chmod +x scripts/install_pytorch_jetson.sh
./scripts/install_pytorch_jetson.sh
```

## What to Expect

The installer will:
1. ✅ Detect your JetPack and CUDA versions
2. ✅ Install system dependencies
3. ✅ Try to download PyTorch wheels from NVIDIA
4. ✅ Install torchvision
5. ✅ Verify CUDA support

**Installation time**: 5-15 minutes (depending on download speed)

## After Installation

Verify everything works:

```bash
# Run verification script
./scripts/verify_pytorch_jetson.sh

# Or test manually
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

## Troubleshooting

If you encounter issues:

1. **Check the troubleshooting guide**: `JETSON_PYTORCH_TROUBLESHOOTING.md`
2. **Verify CUDA**: `nvcc --version`
3. **Check JetPack**: `cat /etc/nv_tegra_release`
4. **Check disk space**: `df -h /`

## Next Steps

Once PyTorch is installed:
- ✅ Run vision processor node: `ros2 run omega_robot vision_processor`
- ✅ Build ROS2 workspace: `cd ~/omega_ws && colcon build`
- ✅ Test multi-device communication

