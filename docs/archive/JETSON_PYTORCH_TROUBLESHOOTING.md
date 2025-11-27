# PyTorch Installation Troubleshooting for Jetson Orin Nano

## 🐛 Common Issues

### Issue 1: `pip install torch` fails with architecture error

**Error**: `ERROR: Could not find a version that satisfies the requirement torch`

**Cause**: Standard PyPI PyTorch wheels are for x86_64, not ARM64/aarch64

**Solution**: Use Jetson-specific installation method (see below)

---

### Issue 2: PyTorch installs but CUDA is not available

**Error**: `torch.cuda.is_available()` returns `False`

**Possible Causes**:
1. PyTorch installed without CUDA support
2. CUDA not properly configured
3. Wrong PyTorch version for your JetPack version

**Solutions**:

1. **Verify CUDA installation**:
```bash
nvcc --version
cat /usr/local/cuda/version.txt  # or
cat /usr/local/cuda/version.json
```

2. **Check JetPack version**:
```bash
cat /etc/nv_tegra_release
```

3. **Reinstall PyTorch with correct version**:
```bash
cd ~/Omega-Code
./scripts/install_pytorch_jetson.sh
```

4. **Reboot and test**:
```bash
sudo reboot
# After reboot
python3 -c "import torch; print(torch.cuda.is_available())"
```

---

### Issue 3: torchvision installation fails

**Error**: `ERROR: Could not build wheels for torchvision`

**Solution**: Install torchvision separately after PyTorch:

```bash
# Method 1: Install compatible torchvision
pip3 install torchvision --no-deps
pip3 install pillow

# Method 2: Build from source (slow)
pip3 install torchvision --no-binary torchvision
```

---

### Issue 4: Out of memory during installation

**Error**: `Killed` or `MemoryError`

**Solution**: 
1. Increase swap space:
```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

2. Install with limited parallelism:
```bash
export MAX_JOBS=1
pip3 install torch torchvision --no-cache-dir
```

---

## ✅ Correct Installation Methods

### Method 1: NVIDIA Pre-built Wheels (Recommended)

```bash
# Run the installation script
cd ~/Omega-Code
./scripts/install_pytorch_jetson.sh
```

This script:
- Detects your JetPack version
- Downloads appropriate PyTorch wheel from NVIDIA
- Installs torchvision
- Verifies CUDA support

### Method 2: Manual Wheel Installation

1. **Find your JetPack version**:
```bash
cat /etc/nv_tegra_release
```

2. **Download appropriate wheel**:
   - Visit: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
   - Find wheel matching your JetPack version
   - Example for JetPack 5.1.2:
```bash
wget https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0-cp38-cp38-linux_aarch64.whl
pip3 install torch-2.1.0-cp38-cp38-linux_aarch64.whl
```

3. **Install torchvision**:
```bash
pip3 install torchvision --no-deps
pip3 install pillow
```

### Method 3: Build from Source (Last Resort)

⚠️ **Warning**: This takes 30-60 minutes and requires significant disk space

```bash
# Install build dependencies
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libjpeg-dev \
    libpng-dev

# Install PyTorch from source
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

---

## 🧪 Verification Steps

### 1. Check PyTorch Installation

```bash
python3 -c "import torch; print(f'PyTorch: {torch.__version__}')"
```

### 2. Check CUDA Availability

```bash
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

### 3. Test GPU Tensor

```bash
python3 << EOF
import torch
if torch.cuda.is_available():
    x = torch.randn(3, 3).cuda()
    print(f"✅ GPU tensor test passed: {x.shape}")
    print(f"✅ Device: {torch.cuda.get_device_name(0)}")
else:
    print("❌ CUDA not available")
EOF
```

### 4. Test torchvision

```bash
python3 -c "import torchvision; print(f'torchvision: {torchvision.__version__}')"
```

---

## 📋 System Requirements Check

Run this diagnostic script:

```bash
#!/bin/bash
echo "=== Jetson PyTorch Diagnostic ==="
echo ""
echo "1. System Info:"
uname -a
echo ""
echo "2. JetPack Version:"
cat /etc/nv_tegra_release 2>/dev/null || echo "Not found"
echo ""
echo "3. CUDA Version:"
nvcc --version 2>/dev/null || echo "CUDA not found"
echo ""
echo "4. Python Version:"
python3 --version
echo ""
echo "5. PyTorch Installation:"
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')" 2>&1 || echo "PyTorch not installed"
echo ""
echo "6. Disk Space:"
df -h /
echo ""
echo "7. Memory:"
free -h
```

---

## 🔧 Quick Fixes

### Fix 1: Reinstall PyTorch

```bash
# Uninstall existing PyTorch
pip3 uninstall torch torchvision torchaudio -y

# Reinstall using Jetson script
cd ~/Omega-Code
./scripts/install_pytorch_jetson.sh
```

### Fix 2: Clear pip cache

```bash
pip3 cache purge
pip3 install --no-cache-dir torch torchvision
```

### Fix 3: Use specific Python version

```bash
# Ensure you're using python3, not python
which python3
python3 --version

# Install with explicit python3
python3 -m pip install torch torchvision
```

### Fix 4: Set environment variables

```bash
# Add to ~/.bashrc
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# Reload
source ~/.bashrc
```

---

## 📚 Additional Resources

- **NVIDIA PyTorch Forum**: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- **JetPack Documentation**: https://developer.nvidia.com/embedded/jetpack
- **PyTorch Official Docs**: https://pytorch.org/get-started/locally/

---

## 🆘 Still Having Issues?

1. **Check logs**: Look for specific error messages
2. **Verify JetPack**: Ensure JetPack is properly installed
3. **Check CUDA**: Verify CUDA toolkit is installed
4. **Disk space**: Ensure you have at least 10GB free
5. **Memory**: Consider increasing swap space

Run diagnostic and share output:
```bash
cd ~/Omega-Code
./scripts/install_pytorch_jetson.sh
```

---

**Last Updated**: 2024  
**Tested On**: Jetson Orin Nano, JetPack 5.1.2

