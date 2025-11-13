# ROS2 Quick Start Guide

Get your ROS2 development environment up and running quickly.

## 🚀 Quick Setup (5 Minutes)

### 1. Build ROS2 Nodes

```bash
# On Lenovo (with ROS2)
cd ~/omega_ws
colcon build --packages-select omega_robot
source install/setup.bash
```

### 2. Test Nodes

```bash
# Terminal 1: Sensor data
ros2 run omega_robot sensor_data_publisher

# Terminal 2: Robot controller
ros2 run omega_robot robot_controller

# Terminal 3: Check topics
ros2 topic list
ros2 topic echo /omega/sensors/ultrasonic
```

### 3. Launch Everything

```bash
# Launch all nodes at once
ros2 launch omega_robot robot_full.launch.py
```

### 4. Connect Web App

```bash
# Start backend (Lenovo with ROS2)
export ROS_NATIVE_MODE=true
cd servers/robot-controller-backend
source venv/bin/activate
python main_api.py

# Web app connects to: ws://localhost:8000/api/ros/bridge
```

## 📡 Common Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/omega/sensors/ultrasonic` | Float32 | Distance in cm |
| `/omega/sensors/line_tracking` | Int32MultiArray | Line sensor readings |
| `/omega/sensors/battery` | BatteryState | Battery voltage/percentage |
| `/omega/motors/left` | Float32 | Left motor speed (-1.0 to 1.0) |
| `/omega/motors/right` | Float32 | Right motor speed (-1.0 to 1.0) |
| `/omega/telemetry` | String | JSON telemetry data |
| `/cmd_vel` | Twist | Velocity commands (from web app) |

## 🎮 Control Robot from Web

```javascript
// Connect to bridge
const ws = new WebSocket('ws://localhost:8000/api/ros/bridge');

// Move forward
ws.send(JSON.stringify({
  type: "publish",
  topic: "/cmd_vel",
  msg_type: "Twist",
  command: {
    linear: { x: 0.5, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  }
}));

// Turn left
ws.send(JSON.stringify({
  type: "publish",
  topic: "/cmd_vel",
  msg_type: "Twist",
  command: {
    linear: { x: 0.3, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0.5 }
  }
}));
```

## 📊 Monitor Sensors

```javascript
// Subscribe to ultrasonic
ws.send(JSON.stringify({
  type: "subscribe",
  topic: "/omega/sensors/ultrasonic",
  msg_type: "Float32"
}));

// Receive data
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === "ros2_message") {
    console.log("Distance:", msg.data.data, "cm");
  }
};
```

## 🔧 Troubleshooting

### Nodes Not Found

```bash
# Rebuild
cd ~/omega_ws
colcon build --packages-select omega_robot
source install/setup.bash
```

### Topics Not Visible

```bash
# Check ROS2 daemon
ros2 daemon stop
ros2 daemon start

# Verify domain ID
echo $ROS_DOMAIN_ID
```

### Web Bridge Not Working

```bash
# Check ROS2 is available
export ROS_NATIVE_MODE=true
source ~/.ros2_rolling_setup.bash

# Verify bridge initialized
# Check backend logs for "ROS2-Web bridge initialized"
```

## 📚 Next Steps

- See `ROS2_EXPANSION_PLAN.md` for full roadmap
- See `ROS2_OPENCV_INTEGRATION.md` for camera integration
- See `ROS2_MULTI_LAPTOP_SETUP.md` for multi-device setup

---

**Happy ROS2 Development!** 🚀

