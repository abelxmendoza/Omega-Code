# Jetson PyTorch Installation Status

## Current Status

**Date**: 2025-01-13  
**System**: Jetson Orin Nano  
**JetPack**: R36 (Revision 4.7)  
**CUDA**: 12.6.68  
**cuDNN**: 9.3.0  
**Python**: 3.10

## Installation Attempts

### ✅ Successfully Installed
- PyTorch 2.0.1 (from PyPI) - **NO CUDA SUPPORT**
- torchvision 0.15.2
- All dependencies (numpy, pillow, etc.)

### ❌ Issues Encountered

1. **Original PyTorch wheel** (`torch-2.4.0a0+3bcc3cddb5.nv24.07.16234504-cp310-cp310-linux_aarch64.whl`)
   - Requires `libcudnn.so.8` (cuDNN 8)
   - System has `libcudnn.so.9` (cuDNN 9)
   - Version symbol mismatch - symlinks don't work

2. **Missing libraries**:
   - `libcusparseLt.so.0` - Not available in CUDA 12.6
   - `libcudnn.so.8` - System has cuDNN 9, not 8

3. **NVIDIA wheel URLs**: Standard URLs returned 404 errors
   - `/jp/v512/pytorch/torch-2.1.0-cp310-cp310-linux_aarch64.whl` - 404
   - `/jp/v511/pytorch/torch-2.1.0-cp310-cp310-linux_aarch64.whl` - 404
   - `/jp/v510/pytorch/torch-2.1.0-cp310-cp310-linux_aarch64.whl` - 404

## What's Needed

A PyTorch wheel built specifically for:
- ✅ JetPack R36 (or 5.1.2)
- ✅ CUDA 12.6
- ✅ cuDNN 9.3.0
- ✅ Python 3.10
- ✅ ARM64/aarch64 architecture

## Next Steps

1. **Check NVIDIA Forums**: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
   - Look for latest PyTorch wheels for JetPack R36
   - Check for cuDNN 9 compatible builds

2. **Check NVIDIA's PyTorch Repository**:
   - The original wheel filename suggests it came from NVIDIA
   - Look for updated versions compatible with cuDNN 9

3. **Alternative**: Use PyTorch without CUDA for now
   - Current installation works for CPU-only operations
   - Can be upgraded when CUDA-compatible wheel is found

## Library Paths Configured

Added to `~/.bashrc`:
```bash
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:/usr/local/cuda/lib64:/usr/local/cuda-12.6/lib64:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-12.6
export PATH=$CUDA_HOME/bin:$PATH
```

## Verification Commands

```bash
# Check PyTorch installation
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"

# Check CUDA
nvcc --version

# Check cuDNN
ls -la /usr/lib/aarch64-linux-gnu/libcudnn.so*

# Check JetPack
cat /etc/nv_tegra_release
```

## Resources

- NVIDIA PyTorch Forum: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- JetPack Documentation: https://developer.nvidia.com/embedded/jetpack
- PyTorch Installation Guide: See `JETSON_PYTORCH_TROUBLESHOOTING.md`

