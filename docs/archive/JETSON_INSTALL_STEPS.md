# Jetson Installation Steps

## Current Status
- ✅ You're on Jetson: `omega1@omega1nano`
- ✅ In directory: `~/Desktop/Omega-Code`
- ✅ Branch: `jetson_setup`
- ✅ PyTorch wheel file detected: `torch-2.4.0a0+3bcc3cddb5.nv24.07.16234504-cp310-cp310-linux_aarch64.whl`

## Next Steps

### 1. Check if scripts exist
```bash
ls -la scripts/install_pytorch_jetson.sh scripts/verify_pytorch_jetson.sh
```

### 2. If scripts exist, run installer
```bash
chmod +x scripts/install_pytorch_jetson.sh scripts/verify_pytorch_jetson.sh
./scripts/install_pytorch_jetson.sh
```

### 3. If scripts don't exist, install PyTorch manually
Since you already have a PyTorch wheel file, you can install it directly:

```bash
# Install the wheel file you have
pip3 install torch-2.4.0a0+3bcc3cddb5.nv24.07.16234504-cp310-cp310-linux_aarch64.whl

# Install torchvision
pip3 install torchvision --no-deps
pip3 install pillow

# Verify installation
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

### 4. Verify installation
```bash
python3 << 'EOF'
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"GPU device: {torch.cuda.get_device_name(0)}")
    x = torch.randn(3, 3).cuda()
    print(f"GPU tensor test: {x.shape}")
EOF
```

