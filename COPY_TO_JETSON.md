# Copy Scripts to Jetson

## Quick Copy Methods

### Option 1: Copy Scripts Directly (Fastest)

From your **laptop/MacBook**, run:

```bash
# Create scripts directory on Jetson
ssh omega1@100.107.112.110 "mkdir -p ~/scripts"

# Copy PyTorch installation scripts
scp scripts/install_pytorch_jetson.sh omega1@100.107.112.110:~/scripts/
scp scripts/verify_pytorch_jetson.sh omega1@100.107.112.110:~/scripts/
```

Then on Jetson:
```bash
cd ~/scripts
chmod +x *.sh
./install_pytorch_jetson.sh
```

### Option 2: Clone Repository (If you have Git access)

On Jetson:
```bash
cd ~
git clone <your-repo-url> Omega-Code
cd Omega-Code
chmod +x scripts/*.sh
./scripts/install_pytorch_jetson.sh
```

### Option 3: Copy Entire Project via rsync

From your laptop:
```bash
rsync -avz --exclude 'venv' --exclude 'node_modules' --exclude '.git' \
  ~/Desktop/Omega-Code/ omega1@100.107.112.110:~/Omega-Code/
```

### Option 4: Manual Copy (Create Scripts on Jetson)

If you can't copy files, you can create the scripts directly on the Jetson.

