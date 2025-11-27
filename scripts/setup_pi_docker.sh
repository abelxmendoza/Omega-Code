#!/bin/bash
# setup_pi_docker.sh
# Set up Docker and ROS2 container on Raspberry Pi
# Run this ON THE PI (via SSH or directly)

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🐳 Setting up Omega ROS2 Docker Environment${NC}"
echo "=============================================="
echo ""

# Check if running on Pi
if [ ! -f /proc/device-tree/model ] || ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo -e "${YELLOW}⚠️  Warning: This script is designed for Raspberry Pi${NC}"
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Step 1: Install Docker
echo -e "${YELLOW}Step 1: Installing Docker...${NC}"
if ! command -v docker &> /dev/null; then
    echo "📦 Installing Docker..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
    sudo usermod -aG docker $USER
    rm get-docker.sh
    echo -e "${GREEN}✅ Docker installed${NC}"
    echo "   ⚠️  Please log out and back in for group changes to take effect"
else
    echo -e "${GREEN}✅ Docker already installed${NC}"
fi

# Step 2: Install Docker Compose
echo ""
echo -e "${YELLOW}Step 2: Installing Docker Compose...${NC}"
if ! command -v docker-compose &> /dev/null; then
    echo "📦 Installing Docker Compose..."
    sudo apt-get update
    sudo apt-get install -y docker-compose-plugin
    echo -e "${GREEN}✅ Docker Compose installed${NC}"
else
    echo -e "${GREEN}✅ Docker Compose already installed${NC}"
fi

# Step 3: Clone/Update Omega-Code
echo ""
echo -e "${YELLOW}Step 3: Setting up Omega-Code repository...${NC}"
OMEGA_CODE_DIR="$HOME/Omega-Code"

if [ ! -d "$OMEGA_CODE_DIR" ]; then
    echo "📥 Cloning Omega-Code repository..."
    cd "$HOME"
    git clone https://github.com/abelxmendoza/Omega-Code.git || {
        echo -e "${RED}❌ Failed to clone repository${NC}"
        echo "   Please check:"
        echo "   1. Repository URL is correct"
        echo "   2. You have access to the repository"
        exit 1
    }
    echo -e "${GREEN}✅ Repository cloned${NC}"
else
    echo "📥 Updating existing repository..."
    cd "$OMEGA_CODE_DIR"
    git pull origin master || echo "⚠️  Git pull failed, continuing..."
    echo -e "${GREEN}✅ Repository updated${NC}"
fi

# Step 4: Build Docker image
echo ""
echo -e "${YELLOW}Step 4: Building Docker image...${NC}"
cd "$OMEGA_CODE_DIR/docker/ros2_robot"

if [ ! -f "docker-compose.dev.yml" ]; then
    echo -e "${RED}❌ docker-compose.dev.yml not found${NC}"
    exit 1
fi

echo "🔨 Building ROS2 development image..."
docker compose -f docker-compose.dev.yml build || {
    echo -e "${RED}❌ Docker build failed${NC}"
    exit 1
}
echo -e "${GREEN}✅ Docker image built${NC}"

# Step 5: Create startup script
echo ""
echo -e "${YELLOW}Step 5: Creating startup scripts...${NC}"

# Create script to start container
cat > "$HOME/start_omega_ros2.sh" << 'EOF'
#!/bin/bash
# Start Omega ROS2 Docker container

cd ~/Omega-Code/docker/ros2_robot

if docker ps | grep -q omega_ros2; then
    echo "✅ Container already running"
    echo "   To enter: docker exec -it omega_ros2 bash"
else
    echo "🚀 Starting Omega ROS2 container..."
    docker compose -f docker-compose.dev.yml up -d
    sleep 2
    echo "✅ Container started"
    echo "   To enter: docker exec -it omega_ros2 bash"
fi
EOF

chmod +x "$HOME/start_omega_ros2.sh"

# Create script to sync code
cat > "$HOME/sync_omega_code.sh" << 'EOF'
#!/bin/bash
# Sync code from GitHub and rebuild ROS2 workspace

cd ~/Omega-Code
echo "📥 Pulling latest code..."
git pull origin master

if docker ps | grep -q omega_ros2; then
    echo "🔨 Rebuilding ROS2 workspace..."
    docker exec omega_ros2 bash -c "
        cd /root/omega_ws && \
        source /opt/ros/humble/setup.bash && \
        colcon build --symlink-install && \
        source install/setup.bash
    "
    echo "✅ Code synced and workspace rebuilt"
else
    echo "⚠️  Container not running. Start it with: ~/start_omega_ros2.sh"
fi
EOF

chmod +x "$HOME/sync_omega_code.sh"

echo -e "${GREEN}✅ Startup scripts created${NC}"

# Step 6: Start container
echo ""
echo -e "${YELLOW}Step 6: Starting container...${NC}"
read -p "Start container now? (y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    "$HOME/start_omega_ros2.sh"
fi

echo ""
echo -e "${GREEN}✅ Setup complete!${NC}"
echo ""
echo "📋 Quick Reference:"
echo "  Start container:  ~/start_omega_ros2.sh"
echo "  Sync code:        ~/sync_omega_code.sh"
echo "  Enter container:  docker exec -it omega_ros2 bash"
echo ""
echo "🔧 Inside container:"
echo "  rebuild-ws    - Rebuild ROS2 workspace"
echo "  sync-code     - Pull latest code from GitHub"
echo "  ros-nodes     - List ROS2 nodes"
echo "  ros-topics    - List ROS2 topics"

