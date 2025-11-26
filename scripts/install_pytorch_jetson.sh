#!/bin/bash
# install_pytorch_jetson.sh
# Install PyTorch and torchvision for Jetson devices (ARM64/aarch64)
# Supports Jetson Orin Nano with JetPack 5.x

set -e

echo "🔥 Installing PyTorch for Jetson Orin Nano"
echo "==========================================="
echo ""

# Detect JetPack version
if [ -f /etc/nv_tegra_release ]; then
    JETPACK_VERSION=$(cat /etc/nv_tegra_release | head -1 | cut -f2 -d ' ' | cut -f1 -d ',')
    echo "Detected JetPack version: $JETPACK_VERSION"
else
    echo "⚠️  Cannot detect JetPack version"
    JETPACK_VERSION="5.1.2"  # Default to common version
fi

# Detect Python version
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
echo "Python version: $PYTHON_VERSION"

# Detect CUDA version
if command -v nvcc &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $5}' | cut -d',' -f1)
    echo "CUDA version: $CUDA_VERSION"
else
    echo "❌ CUDA not found. PyTorch requires CUDA on Jetson."
    exit 1
fi

# Upgrade pip
echo ""
echo "📦 Upgrading pip..."
pip3 install --upgrade pip setuptools wheel

# Install system dependencies
echo ""
echo "📦 Installing system dependencies..."
sudo apt update
sudo apt install -y \
    libopenblas-base \
    libopenmpi-dev \
    libomp-dev \
    python3-dev \
    python3-pip

# Method 1: Try NVIDIA's pre-built wheels (recommended)
echo ""
echo "🔍 Attempting to install PyTorch from NVIDIA wheels..."
echo "This method uses pre-built wheels optimized for Jetson"
echo ""
echo "📋 For the latest PyTorch wheels, visit:"
echo "   https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
echo ""

# Determine PyTorch version based on JetPack/CUDA
# JetPack 5.x typically uses PyTorch 2.x
PYTORCH_VERSION="2.1.0"
TORCHVISION_VERSION="0.16.0"

# Try multiple common wheel URLs for different JetPack versions
echo "Attempting to install PyTorch $PYTORCH_VERSION..."

# Install numpy first (required dependency)
pip3 install --upgrade numpy

# Try installing from NVIDIA's wheelhouse (multiple JetPack versions)
WHEEL_INSTALLED=false

# Try JetPack 5.1.2 (common version)
echo "Trying JetPack 5.1.2 wheel..."
pip3 install --no-cache-dir \
    https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-${PYTORCH_VERSION}-cp38-cp38-linux_aarch64.whl \
    2>&1 | grep -q "Successfully installed" && WHEEL_INSTALLED=true || true

# Try JetPack 5.1.1
if [ "$WHEEL_INSTALLED" = false ]; then
    echo "Trying JetPack 5.1.1 wheel..."
    pip3 install --no-cache-dir \
        https://developer.download.nvidia.com/compute/redist/jp/v511/pytorch/torch-${PYTORCH_VERSION}-cp38-cp38-linux_aarch64.whl \
        2>&1 | grep -q "Successfully installed" && WHEEL_INSTALLED=true || true
fi

# Try JetPack 5.1.0
if [ "$WHEEL_INSTALLED" = false ]; then
    echo "Trying JetPack 5.1.0 wheel..."
    pip3 install --no-cache-dir \
        https://developer.download.nvidia.com/compute/redist/jp/v510/pytorch/torch-${PYTORCH_VERSION}-cp38-cp38-linux_aarch64.whl \
        2>&1 | grep -q "Successfully installed" && WHEEL_INSTALLED=true || true
fi

# If all wheel downloads failed, offer alternatives
if [ "$WHEEL_INSTALLED" = false ]; then
    echo ""
    echo "⚠️  Direct wheel download failed"
    echo ""
    echo "📥 Manual Installation Options:"
    echo ""
    echo "Option 1: Download wheel manually"
    echo "  1. Visit: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
    echo "  2. Find wheel matching your JetPack version"
    echo "  3. Download and install:"
    echo "     wget <wheel_url>"
    echo "     pip3 install <wheel_file>.whl"
    echo ""
    echo "Option 2: Use alternative installation method"
    read -p "Try alternative installation? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Exiting. Please install PyTorch manually."
        exit 1
    fi
    
    # Method 2: Install from PyPI with --no-binary (builds from source, slower)
    echo ""
    echo "📦 Installing PyTorch from source (this may take 30-60 minutes)..."
    echo "⚠️  This requires significant disk space and time"
    
    read -p "Continue with source build? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        # Install build dependencies
        sudo apt install -y \
            build-essential \
            cmake \
            git \
            libjpeg-dev \
            libpng-dev
        
        # Install PyTorch from source
        pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
    else
        echo "Skipping PyTorch installation"
        echo ""
        echo "Alternative: Install PyTorch manually:"
        echo "1. Visit: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048"
        echo "2. Download appropriate wheel for your JetPack version"
        echo "3. Install: pip3 install <wheel_file>.whl"
        exit 0
    fi
}

# Verify installation
echo ""
echo "🧪 Verifying PyTorch installation..."
python3 << EOF
import sys
try:
    import torch
    print(f"✅ PyTorch version: {torch.__version__}")
    
    # Check CUDA availability
    if torch.cuda.is_available():
        print(f"✅ CUDA available: {torch.version.cuda}")
        print(f"✅ GPU device: {torch.cuda.get_device_name(0)}")
        print(f"✅ CUDA device count: {torch.cuda.device_count()}")
        
        # Test tensor on GPU
        x = torch.randn(3, 3).cuda()
        print(f"✅ GPU tensor test passed: {x.shape}")
    else:
        print("⚠️  CUDA not available in PyTorch")
        print("   PyTorch installed but GPU support may be missing")
    
    # Check torchvision
    try:
        import torchvision
        print(f"✅ torchvision version: {torchvision.__version__}")
    except ImportError:
        print("⚠️  torchvision not installed")
        print("   Installing torchvision...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "torchvision"])
    
except ImportError as e:
    print(f"❌ PyTorch import failed: {e}")
    sys.exit(1)
EOF

# Install torchvision if not already installed
echo ""
echo "📦 Installing torchvision..."
pip3 install --no-cache-dir torchvision || {
    echo "⚠️  torchvision installation failed"
    echo "   You may need to install it manually"
}

# Install additional ML libraries
echo ""
echo "📦 Installing additional ML libraries..."
pip3 install --no-cache-dir \
    ultralytics \
    transformers \
    pillow \
    scipy

# Summary
echo ""
echo "========================================="
echo "✅ PyTorch installation complete!"
echo ""
echo "📋 Verification:"
echo ""
echo "Run this to test:"
echo "  python3 -c \"import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')\""
echo ""
echo "If CUDA is False, try:"
echo "  1. Reboot: sudo reboot"
echo "  2. Check CUDA: nvcc --version"
echo "  3. Check JetPack: cat /etc/nv_tegra_release"
echo ""
echo "========================================="

