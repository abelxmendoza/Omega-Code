#!/bin/bash
# verify_pytorch_jetson.sh
# Verify PyTorch installation on Jetson Orin Nano

echo "🧪 PyTorch Installation Verification"
echo "==================================="
echo ""

# Check Python
echo "1. Python Version:"
python3 --version
echo ""

# Check CUDA
echo "2. CUDA Installation:"
if command -v nvcc &> /dev/null; then
    nvcc --version | grep "release"
    CUDA_AVAILABLE=true
else
    echo "❌ CUDA not found"
    CUDA_AVAILABLE=false
fi
echo ""

# Check JetPack
echo "3. JetPack Version:"
if [ -f /etc/nv_tegra_release ]; then
    cat /etc/nv_tegra_release
else
    echo "⚠️  JetPack version file not found"
fi
echo ""

# Check PyTorch
echo "4. PyTorch Installation:"
python3 << 'EOF'
import sys
try:
    import torch
    print(f"✅ PyTorch installed: {torch.__version__}")
    
    # Check CUDA
    if torch.cuda.is_available():
        print(f"✅ CUDA available: {torch.version.cuda}")
        print(f"✅ GPU device: {torch.cuda.get_device_name(0)}")
        print(f"✅ CUDA device count: {torch.cuda.device_count()}")
        
        # Test GPU tensor
        try:
            x = torch.randn(3, 3).cuda()
            print(f"✅ GPU tensor test passed: {x.shape}")
        except Exception as e:
            print(f"❌ GPU tensor test failed: {e}")
    else:
        print("⚠️  CUDA not available in PyTorch")
        print("   PyTorch may be installed without CUDA support")
    
    # Check torchvision
    try:
        import torchvision
        print(f"✅ torchvision installed: {torchvision.__version__}")
    except ImportError:
        print("⚠️  torchvision not installed")
    
except ImportError as e:
    print(f"❌ PyTorch not installed: {e}")
    sys.exit(1)
EOF

PYTORCH_STATUS=$?

echo ""
echo "5. System Resources:"
echo "   Disk space:"
df -h / | tail -1
echo "   Memory:"
free -h | grep Mem

echo ""
echo "==================================="
if [ $PYTORCH_STATUS -eq 0 ]; then
    echo "✅ Verification complete"
else
    echo "❌ PyTorch verification failed"
    echo ""
    echo "Next steps:"
    echo "  1. Run: ./scripts/install_pytorch_jetson.sh"
    echo "  2. See: JETSON_PYTORCH_TROUBLESHOOTING.md"
fi
echo "==================================="

