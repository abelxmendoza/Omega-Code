# AutonomyModal ROS2 Integration Guide

Complete guide to using ROS2 with the AutonomyModal - your single control center for autonomous robot behaviors.

## 🎯 Overview

The **AutonomyModal** is now fully integrated with ROS2. When you enable ROS2 integration, the modal automatically uses ROS2 action servers for autonomous behaviors instead of simple logging modes.

## ✅ What's Integrated

### Frontend (AutonomyModal)
- ✅ **Real-time ROS2 status checking** - Checks every 3 seconds when enabled
- ✅ **ROS2 connection badge** - Green ✓ when available, red ✗ when not
- ✅ **Mode indicators** - Shows 🚀 emoji when mode uses ROS2
- ✅ **Status messages** - Shows ROS2 mode (native/docker) and topic count
- ✅ **ROS2 Domain ID input** - Configure communication domain (0-232)

### Backend
- ✅ **ROS2 mode handlers** - Automatically used when ROS2 available
- ✅ **ROS2 context passing** - Bridge passed to mode handlers
- ✅ **Auto-registration** - ROS2 modes register automatically
- ✅ **Graceful fallback** - Uses simple modes if ROS2 unavailable

## 🚀 How to Use

### 1. Enable ROS2 Integration

1. Open **AutonomyModal** (click "Autonomy" button)
2. Scroll to "ROS Controls & Features" section
3. Toggle **"Enable ROS2 Integration"** to ON
4. Check the **ROS2 badge** - should turn green ✓

### 2. Check ROS2 Status

When ROS2 Integration is enabled, you'll see:
- **Green badge**: ROS2 available (native or docker mode)
- **Red badge**: ROS2 not available
- **Status message**: Shows mode type and topic count

### 3. Select Autonomous Mode

Choose a mode that supports ROS2:
- **Line Follow** 🚀 - Uses ROS2 `follow_line` action server
- **Avoid Obstacles** 🚀 - Uses ROS2 `obstacle_avoidance` action server  
- **Waypoints** 🚀 - Uses ROS2 `navigate_to_goal` action server

The 🚀 emoji indicates the mode will use ROS2 when available.

### 4. Start Autonomy

1. Configure any parameters (speed, duration, etc.)
2. Click **"Start"**
3. The backend automatically:
   - Uses ROS2 mode handler if ROS2 available
   - Publishes to ROS2 topics (`/cmd_vel`)
   - Connects to ROS2 action servers

## 📊 ROS2 Status Indicators

### Badge Colors
- **Green (✓)**: ROS2 connected and ready
- **Red (✗)**: ROS2 not available

### Status Messages
- `ROS2 native mode connected` - Using native ROS2 on laptop
- `ROS2 docker mode connected` - Using Docker ROS2 on Pi
- `ROS2 not available` - ROS2 not detected

### Mode Descriptions
- **Without ROS2**: "Follows marked lines"
- **With ROS2**: "Uses ROS2 follow_line action server 🚀"

## 🔧 Configuration

### ROS2 Domain ID

Set the ROS2 Domain ID (0-232) to control which devices can communicate:
- **Same Domain ID**: Devices can see each other's topics
- **Different Domain ID**: Devices are isolated

Default: `0`

### ROS2 Toggles

When ROS2 Integration is enabled, you can control:
- **Publish Sensors**: Send sensor data to ROS2 topics
- **Publish Movement**: Send movement commands to ROS2
- **Publish Camera**: Send camera feed to ROS2
- **Subscribe Commands**: Receive commands from ROS2
- **ROS Nodes**: Enable SLAM, sensor fusion, path planning
- **ROS Bridge**: Enable WebSocket ↔ ROS2 conversion
- **TF (Transforms)**: Enable coordinate transforms

## 🎮 Mode Behavior

### Line Follow Mode
**Without ROS2**: Logs to console
**With ROS2**: 
- Publishes to `/cmd_vel` for movement
- Can use `follow_line` action server
- Reads from `/omega/sensors/line_tracking`

### Avoid Obstacles Mode
**Without ROS2**: Logs to console
**With ROS2**:
- Publishes to `/cmd_vel` for movement
- Can use `obstacle_avoidance` action server
- Reads from `/omega/sensors/ultrasonic`

### Waypoints Mode
**Without ROS2**: Logs waypoints
**With ROS2**:
- Publishes to `/cmd_vel` for navigation
- Can use `navigate_to_goal` action server
- Uses odometry from `/odom`

## 🔍 Troubleshooting

### ROS2 Badge Shows Red ✗

**Possible causes:**
1. ROS2 not installed on laptop
2. `ROS_NATIVE_MODE` not set
3. ROS2 daemon not running
4. Backend can't connect to ROS2

**Solutions:**
```bash
# Check ROS2 installation
ros2 --help

# Set environment variable
export ROS_NATIVE_MODE=true
source /opt/ros/rolling/setup.bash

# Restart backend
cd servers/robot_controller_backend
python main_api.py
```

### Modes Not Using ROS2

**Check:**
1. ROS2 Integration toggle is ON
2. ROS2 badge is green ✓
3. Backend logs show "ROS2 bridge available"
4. ROS2 action servers are running

**Verify:**
```bash
# Check ROS2 nodes
ros2 node list

# Check action servers
ros2 action list

# Check topics
ros2 topic list
```

### Mode Starts But Robot Doesn't Move

**Possible causes:**
1. ROS2 action servers not running
2. `robot_controller` node not running
3. Hardware not connected

**Solutions:**
```bash
# Launch ROS2 nodes
ros2 launch omega_robot robot_full.launch.py

# Check cmd_vel topic
ros2 topic echo /cmd_vel
```

## 📈 Advanced Usage

### Custom ROS2 Parameters

You can pass ROS2-specific parameters in the mode params:
```javascript
{
  rosEnabled: true,
  duration: 10.0,  // For line follow
  direction: "forward",  // For obstacle avoidance
  goal_x: 1.0,  // For waypoints
  goal_y: 0.5
}
```

### Multiple Devices

When using multiple devices (Laptop + Pi):
1. Set same ROS2 Domain ID on both
2. Ensure CycloneDDS config has peer addresses
3. AutonomyModal on laptop can control Pi's ROS2 nodes

## 🎓 Best Practices

1. **Always check ROS2 status** before starting autonomy
2. **Use ROS2 Domain ID** to isolate test environments
3. **Monitor ROS2 topics** while autonomy is running
4. **Check backend logs** for ROS2 connection status
5. **Start ROS2 nodes first** before enabling autonomy

## 🔗 Related Components

**Note**: These are separate and optional:
- `AutonomousActions` component - Direct action control (can ignore)
- `ROS2TopicViewer` - Topic monitoring (can ignore)
- `CameraViewer` - Camera display (can ignore)
- `MapViewer` - Map visualization (can ignore)

**Focus on AutonomyModal** - it's your main control center!

---

**Status**: Fully Integrated ✅  
**Last Updated**: 2024

