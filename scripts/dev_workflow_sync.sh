#!/bin/bash
# dev_workflow_sync.sh
# Sync code from MacBook → GitHub → Pi Docker container
# Usage: ./scripts/dev_workflow_sync.sh [pi-hostname]

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Pi hostname (default: omega1-tailscale or omega1)
PI_HOST="${1:-omega1-tailscale}"

echo -e "${BLUE}🔄 Omega Development Workflow Sync${NC}"
echo "=========================================="
echo ""

# Step 1: Commit and push from MacBook
echo -e "${YELLOW}Step 1: Committing and pushing to GitHub...${NC}"
cd "$PROJECT_ROOT"

# Check if there are uncommitted changes
if [ -n "$(git status --porcelain)" ]; then
    echo "📝 Uncommitted changes detected:"
    git status --short
    
    read -p "Commit and push? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        read -p "Commit message: " COMMIT_MSG
        git add -A
        git commit -m "${COMMIT_MSG:-Update code}"
        git push origin master
        echo -e "${GREEN}✅ Pushed to GitHub${NC}"
    else
        echo "⚠️  Skipping commit. Continuing with existing commits..."
    fi
else
    echo "✅ No uncommitted changes"
fi

# Step 2: Pull on Pi
echo ""
echo -e "${YELLOW}Step 2: Pulling latest code on Pi ($PI_HOST)...${NC}"

# Check if we can SSH to Pi
if ! ssh -o ConnectTimeout=5 "$PI_HOST" "echo 'Connected'" > /dev/null 2>&1; then
    echo -e "❌ Cannot connect to $PI_HOST"
    echo "   Please check:"
    echo "   1. Pi is powered on and connected"
    echo "   2. SSH key is set up"
    echo "   3. Hostname is correct"
    exit 1
fi

# Pull code on Pi
ssh "$PI_HOST" << 'EOF'
cd /home/omega1/Omega-Code
echo "📥 Pulling latest code from GitHub..."
git pull origin master || {
    echo "⚠️  Git pull failed. Checking status..."
    git status
    exit 1
}
echo "✅ Code updated on Pi"
EOF

# Step 3: Rebuild ROS2 workspace in Docker container
echo ""
echo -e "${YELLOW}Step 3: Rebuilding ROS2 workspace in Docker container...${NC}"

ssh "$PI_HOST" << 'EOF'
# Check if container is running
if ! docker ps | grep -q omega_ros2; then
    echo "⚠️  Container omega_ros2 is not running"
    echo "   Starting container..."
    cd /home/omega1/Omega-Code/docker/ros2_robot
    docker-compose -f docker-compose.dev.yml up -d
    sleep 3
fi

# Rebuild workspace inside container
echo "🔨 Rebuilding ROS2 workspace..."
docker exec -it omega_ros2 bash -c "
    cd /root/omega_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    echo '✅ Workspace rebuilt successfully'
" || {
    echo "❌ Workspace rebuild failed"
    exit 1
}
EOF

echo ""
echo -e "${GREEN}✅ Sync complete!${NC}"
echo ""
echo "Next steps:"
echo "  1. SSH to Pi: ssh $PI_HOST"
echo "  2. Enter container: docker exec -it omega_ros2 bash"
echo "  3. Run ROS2 nodes: ros2 run omega_robot <node_name>"

