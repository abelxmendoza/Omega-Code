# Fix cuDNN Library Path on Jetson

## Issue
```
ImportError: libcudnn.so.8: cannot open shared object file: No such file or directory
```

## Solution

### Step 1: Find cuDNN library location

```bash
# Search for cuDNN libraries
find /usr -name "libcudnn.so*" 2>/dev/null
```

### Step 2: Add to library path

Add cuDNN to your environment. Run these commands:

```bash
# Check where cuDNN is located
ls -la /usr/lib/aarch64-linux-gnu/libcudnn* 2>/dev/null || \
ls -la /usr/local/cuda*/lib64/libcudnn* 2>/dev/null || \
find /usr -name "libcudnn.so*" 2>/dev/null
```

### Step 3: Set LD_LIBRARY_PATH

Add to `~/.bashrc`:

```bash
# Add cuDNN to library path
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# Or if cuDNN is in a different location, add that path
```

### Step 4: Reload and test

```bash
source ~/.bashrc
python3 -c "import torch; print(f'PyTorch: {torch.__version__}'); print(f'CUDA: {torch.cuda.is_available()}')"
```

