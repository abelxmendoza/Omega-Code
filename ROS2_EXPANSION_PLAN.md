# ROS2 Expansion Plan for Omega Robot

This document outlines the comprehensive expansion of ROS2 integration for the Omega robot and web application.

## 🎯 Goals

1. **Real-time sensor data streaming** from Pi to web app
2. **Robot control** from web app via ROS2
3. **Autonomous behaviors** with ROS2 action servers
4. **Visualization** of robot state in web UI
5. **Recording/playback** of robot sessions
6. **Multi-robot support** for fleet management

## 📦 New ROS2 Nodes Created

### 1. Sensor Data Publisher (`sensor_data_publisher`)

**Purpose**: Publishes comprehensive sensor data from hardware

**Topics Published**:
- `/omega/sensors/ultrasonic` (Float32) - Distance in cm
- `/omega/sensors/line_tracking` (Int32MultiArray) - Line sensor readings
- `/omega/sensors/battery` (BatteryState) - Battery voltage and percentage
- `/omega/system/ready` (Bool) - System readiness status

**Usage**:
```bash
ros2 run omega_robot sensor_data_publisher
```

### 2. Robot Controller (`robot_controller`)

**Purpose**: Converts cmd_vel commands to motor control

**Topics**:
- **Subscribes**: `/cmd_vel` (Twist) - Velocity commands
- **Publishes**: 
  - `/omega/motors/left` (Float32) - Left motor speed
  - `/omega/motors/right` (Float32) - Right motor speed
  - `/omega/controller/status` (String) - Controller status

**Usage**:
```bash
ros2 run omega_robot robot_controller
```

**Control from web app**:
```javascript
// Publish cmd_vel command
websocket.send(JSON.stringify({
  type: "publish",
  topic: "/cmd_vel",
  msg_type: "Twist",
  command: {
    linear: { x: 0.5, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0.2 }
  }
}));
```

### 3. Enhanced Telemetry (`enhanced_telemetry`)

**Purpose**: Aggregates sensor data and system info into comprehensive telemetry

**Topics**:
- **Subscribes**: Sensor topics
- **Publishes**: `/omega/telemetry` (String) - JSON telemetry data

**Telemetry Format**:
```json
{
  "timestamp": 1234567890.123,
  "sensors": {
    "ultrasonic_cm": 45.2,
    "line_tracking": [0, 1, 0],
    "battery": {
      "voltage": 7.4,
      "percentage": 85.0
    }
  },
  "system": {
    "cpu_percent": 25.5,
    "memory_percent": 45.2,
    "memory_available_mb": 512.0
  },
  "status": "active"
}
```

## 🌐 ROS2-Web Bridge

### WebSocket Endpoint: `/api/ros/bridge`

**Purpose**: Real-time bidirectional communication between ROS2 and web app

**Features**:
- Subscribe to ROS2 topics from web app
- Publish commands to ROS2 topics from web app
- Real-time message streaming
- Automatic reconnection

**Usage**:
```javascript
const ws = new WebSocket('ws://localhost:8000/api/ros/bridge');

// Subscribe to sensor data
ws.send(JSON.stringify({
  type: "subscribe",
  topic: "/omega/sensors/ultrasonic",
  msg_type: "Float32"
}));

// Receive messages
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === "ros2_message") {
    console.log("Sensor reading:", msg.data);
  }
};

// Publish command
ws.send(JSON.stringify({
  type: "publish",
  topic: "/cmd_vel",
  msg_type: "Twist",
  command: {
    linear: { x: 0.5, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  }
}));
```

## 🚀 Implementation Roadmap

### Phase 1: Core Integration ✅
- [x] Sensor data publisher node
- [x] Robot controller node
- [x] Enhanced telemetry
- [x] ROS2-Web bridge service
- [x] WebSocket endpoint

### Phase 2: Camera Integration
- [ ] Camera image publisher node
- [ ] Compressed image topics
- [ ] Image streaming to web app
- [ ] Camera control (zoom, focus)

### Phase 3: Autonomous Behaviors
- [ ] ROS2 action servers for:
  - Navigate to goal
  - Follow line
  - Obstacle avoidance
  - Patrol route
- [ ] Behavior tree integration
- [ ] Mission planning

### Phase 4: Navigation & SLAM
- [ ] Odometry publisher
- [ ] SLAM integration (gmapping/cartographer)
- [ ] Path planning (A*, D*, RRT)
- [ ] Map visualization in web UI

### Phase 5: Advanced Features
- [ ] Bag file recording/playback
- [ ] Diagnostics and health monitoring
- [ ] Parameter server integration
- [ ] Multi-robot coordination

## 📡 Topic Architecture

```
┌─────────────────────────────────────────────────┐
│              ROS2 Topic Structure               │
├─────────────────────────────────────────────────┤
│                                                 │
│  /omega/sensors/                                │
│    ├── ultrasonic (Float32)                    │
│    ├── line_tracking (Int32MultiArray)         │
│    └── battery (BatteryState)                  │
│                                                 │
│  /omega/motors/                                 │
│    ├── left (Float32)                          │
│    └── right (Float32)                         │
│                                                 │
│  /omega/controller/                             │
│    └── status (String)                         │
│                                                 │
│  /omega/telemetry (String)                      │
│                                                 │
│  /cmd_vel (Twist) ← Web App Commands           │
│                                                 │
└─────────────────────────────────────────────────┘
```

## 🔧 Setup Instructions

### 1. Build New Nodes

```bash
# On Lenovo or Pi
cd ~/omega_ws
colcon build --packages-select omega_robot
source install/setup.bash
```

### 2. Launch Nodes

**Option A: Individual nodes**
```bash
# Terminal 1: Sensors
ros2 run omega_robot sensor_data_publisher

# Terminal 2: Controller
ros2 run omega_robot robot_controller

# Terminal 3: Enhanced telemetry
ros2 run omega_robot enhanced_telemetry
```

**Option B: Launch file**
```bash
ros2 launch omega_robot robot_full.launch.py
```

### 3. Enable Web Bridge

**On Lenovo (with ROS2)**:
```bash
export ROS_NATIVE_MODE=true
cd servers/robot_controller_backend
python main_api.py
```

**On MacBook (without ROS2)**:
```bash
# Bridge will connect to Pi Docker ROS2
export PI_SSH_HOST=pi@192.168.1.107
cd servers/robot_controller_backend
python main_api.py
```

## 🎨 Web UI Integration

### React Components to Create

1. **ROS2TopicViewer** - Display topic data in real-time
2. **RobotController** - Joystick/buttons for cmd_vel
3. **SensorDashboard** - Visualize all sensor readings
4. **TelemetryMonitor** - System health and telemetry
5. **TopicBrowser** - Browse and subscribe to any topic

### Example Component

```tsx
import { useEffect, useState } from 'react';
import { useWebSocket } from '@/hooks/useWebSocket';

export function UltrasonicSensor() {
  const [distance, setDistance] = useState<number | null>(null);
  const ws = useWebSocket('/api/ros/bridge');

  useEffect(() => {
    if (ws) {
      // Subscribe to ultrasonic topic
      ws.send(JSON.stringify({
        type: "subscribe",
        topic: "/omega/sensors/ultrasonic",
        msg_type: "Float32"
      }));

      // Handle messages
      ws.onmessage = (event) => {
        const msg = JSON.parse(event.data);
        if (msg.type === "ros2_message" && msg.topic === "/omega/sensors/ultrasonic") {
          setDistance(msg.data.data);
        }
      };
    }
  }, [ws]);

  return (
    <div>
      <h3>Ultrasonic Sensor</h3>
      <p>Distance: {distance !== null ? `${distance.toFixed(1)} cm` : 'No data'}</p>
    </div>
  );
}
```

## 📊 Monitoring & Debugging

### View Topics

```bash
# List all topics
ros2 topic list

# Echo topic
ros2 topic echo /omega/sensors/ultrasonic

# Get topic info
ros2 topic info /cmd_vel

# Get topic type
ros2 topic type /omega/telemetry
```

### Monitor Node Status

```bash
# List nodes
ros2 node list

# Node info
ros2 node info /sensor_data_publisher
```

### Record Session

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /omega/sensors/ultrasonic /cmd_vel

# Playback
ros2 bag play <bag_file>
```

## 🔐 Security Considerations

1. **Topic Namespace**: Use `/omega/` prefix for all custom topics
2. **Command Validation**: Validate cmd_vel commands before publishing
3. **Rate Limiting**: Limit command publishing rate
4. **Authentication**: Add auth to WebSocket connections
5. **Network Isolation**: Use ROS_DOMAIN_ID for isolation

## 🧪 Testing

### Test Sensor Publisher

```bash
# Run node
ros2 run omega_robot sensor_data_publisher

# In another terminal, echo topics
ros2 topic echo /omega/sensors/ultrasonic
ros2 topic echo /omega/sensors/battery
```

### Test Robot Controller

```bash
# Run controller
ros2 run omega_robot robot_controller

# Send cmd_vel command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Monitor motor commands
ros2 topic echo /omega/motors/left
```

### Test Web Bridge

```bash
# Start backend
python main_api.py

# Test with wscat
wscat -c ws://localhost:8000/api/ros/bridge

# Send subscribe message
{"type": "subscribe", "topic": "/omega/sensors/ultrasonic", "msg_type": "Float32"}
```

## 📈 Performance Optimization

1. **QoS Profiles**: Use appropriate QoS for each topic
2. **Message Frequency**: Limit publishing rates (10-30 Hz)
3. **Compression**: Use compressed image topics for camera
4. **Selective Subscription**: Only subscribe to needed topics
5. **Caching**: Cache sensor readings in web app

## 🎓 Next Steps

1. **Implement camera publisher** - Stream camera to ROS2
2. **Create action servers** - Autonomous behaviors
3. **Add web UI components** - Visualize ROS2 data
4. **Integrate navigation** - SLAM and path planning
5. **Add diagnostics** - System health monitoring

---

**Last Updated**: 2024  
**Status**: Phase 1 Complete ✅  
**Next**: Phase 2 - Camera Integration

