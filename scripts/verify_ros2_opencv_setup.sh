#!/bin/bash
# verify_ros2_opencv_setup.sh
# Verifies ROS2 and OpenCV are properly set up for the robot controller

set -e

echo "🔍 Verifying ROS2 and OpenCV Setup"
echo "===================================="
echo ""

ERRORS=0
WARNINGS=0

# Check ROS2
echo "📦 Checking ROS2..."
if [ -f /opt/ros/rolling/setup.bash ]; then
    echo "  ✅ ROS2 Rolling found"
    source /opt/ros/rolling/setup.bash 2>/dev/null || true
    
    if command -v ros2 &> /dev/null; then
        echo "  ✅ ros2 command available"
        ROS_VERSION=$(ros2 --version 2>&1 | head -1 || echo "unknown")
        echo "  📋 $ROS_VERSION"
    else
        echo "  ❌ ros2 command not found"
        ((ERRORS++))
    fi
    
    # Check rclpy
    if python3 -c "import rclpy" 2>/dev/null; then
        echo "  ✅ rclpy Python module available"
    else
        echo "  ⚠️  rclpy not available (install: sudo apt install ros-rolling-rclpy)"
        ((WARNINGS++))
    fi
else
    echo "  ❌ ROS2 Rolling not found at /opt/ros/rolling"
    ((ERRORS++))
fi

echo ""

# Check OpenCV
echo "📷 Checking OpenCV..."
if python3 -c "import cv2; print(f'OpenCV {cv2.__version__}')" 2>/dev/null; then
    OPENCV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null)
    echo "  ✅ OpenCV $OPENCV_VERSION available"
    
    # Check OpenCV contrib
    if python3 -c "import cv2; cv2.aruco" 2>/dev/null; then
        echo "  ✅ OpenCV contrib features available"
    else
        echo "  ⚠️  OpenCV contrib features not available"
        echo "     Install: pip install opencv-contrib-python"
        ((WARNINGS++))
    fi
else
    echo "  ❌ OpenCV not available"
    echo "     Install: pip install opencv-python opencv-contrib-python"
    ((ERRORS++))
fi

echo ""

# Check workspace
echo "📁 Checking ROS2 workspace..."
if [ -d ~/omega_ws ]; then
    echo "  ✅ Workspace found at ~/omega_ws"
    
    if [ -f ~/omega_ws/install/setup.bash ]; then
        echo "  ✅ Workspace built"
        
        if [ -d ~/omega_ws/src/omega_robot ]; then
            echo "  ✅ omega_robot package found"
        else
            echo "  ⚠️  omega_robot package not found in workspace"
            ((WARNINGS++))
        fi
    else
        echo "  ⚠️  Workspace not built (run: cd ~/omega_ws && colcon build)"
        ((WARNINGS++))
    fi
else
    echo "  ⚠️  Workspace not found at ~/omega_ws"
    ((WARNINGS++))
fi

echo ""

# Check backend dependencies
echo "🐍 Checking Python dependencies..."
cd "$(dirname "$0")/../servers/robot-controller-backend" 2>/dev/null || cd /home/abelxmendoza/Desktop/code/Omega-Code/servers/robot-controller-backend

if [ -f "requirements.txt" ]; then
    echo "  ✅ requirements.txt found"
    
    # Check if venv exists
    if [ -d "venv" ]; then
        echo "  ✅ Virtual environment found"
        source venv/bin/activate 2>/dev/null || true
        
        if python3 -c "import cv2" 2>/dev/null; then
            echo "  ✅ OpenCV available in venv"
        else
            echo "  ⚠️  OpenCV not in venv (run: pip install opencv-python)"
            ((WARNINGS++))
        fi
    else
        echo "  ⚠️  Virtual environment not found"
        echo "     Create: python3 -m venv venv"
        ((WARNINGS++))
    fi
else
    echo "  ⚠️  requirements.txt not found"
    ((WARNINGS++))
fi

echo ""

# Check project paths
echo "📂 Checking project configuration..."
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
echo "  📍 Project root: $PROJECT_ROOT"

if [ -f "$PROJECT_ROOT/docker/ros2_robot/docker-compose.yml" ]; then
    echo "  ✅ Docker Compose config found"
else
    echo "  ⚠️  Docker Compose config not found"
    ((WARNINGS++))
fi

if [ -f "$PROJECT_ROOT/docker/ros2_robot/config/cyclonedds.xml" ]; then
    echo "  ✅ CycloneDDS config found"
else
    echo "  ⚠️  CycloneDDS config not found"
    ((WARNINGS++))
fi

echo ""

# Summary
echo "===================================="
if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo "✅ All checks passed!"
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo "⚠️  Setup complete with $WARNINGS warning(s)"
    echo "   Review warnings above"
    exit 0
else
    echo "❌ Setup incomplete: $ERRORS error(s), $WARNINGS warning(s)"
    echo "   Please fix errors before using the robot controller"
    exit 1
fi

