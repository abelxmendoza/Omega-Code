# ROS2 Architecture for Omega Robot

Complete architecture diagram and data flow for ROS2 integration.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Omega Robot ROS2 Ecosystem                       │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────────┐         ┌──────────────┐    ┌──────────────┐    │
│  │   Lenovo     │         │   MacBook    │    │  Raspberry   │    │
│  │  (Ubuntu)    │         │   (macOS)    │    │     Pi 4B    │    │
│  │              │         │              │    │              │    │
│  │ ┌──────────┐ │         │ ┌──────────┐ │    │ ┌──────────┐ │    │
│  │ │  ROS2    │ │         │ │  Web UI  │ │    │ │  Docker  │ │    │
│  │ │ Rolling  │ │         │ │  (Next)  │ │    │ │  ROS2    │ │    │
│  │ │ Native   │ │         │ │          │ │    │ │  Humble  │ │    │
│  │ └────┬─────┘ │         │ └────┬─────┘ │    │ └────┬─────┘ │    │
│  │      │       │         │      │       │    │      │       │    │
│  │ ┌────▼─────┐ │         │ ┌────▼─────┐ │    │      │       │    │
│  │ │ Backend  │ │         │ │ Backend  │ │    │      │       │    │
│  │ │ FastAPI  │ │         │ │ FastAPI  │ │    │      │       │    │
│  │ │          │ │         │ │          │ │    │      │       │    │
│  │ │ ROS2-Web │ │         │ │ ROS2-Web │ │    │      │       │    │
│  │ │ Bridge   │ │         │ │ Bridge   │ │    │      │       │    │
│  │ └────┬─────┘ │         │ └────┬─────┘ │    │      │       │    │
│  └──────┼───────┘         └──────┼───────┘    └──────┼───────┘    │
│         │                        │                    │            │
│         └────────────────────────┼────────────────────┘            │
│                                  │                                 │
│                    ┌─────────────▼─────────────┐                   │
│                    │   ROS2 DDS Network        │                   │
│                    │   (CycloneDDS)            │                   │
│                    │   Domain ID: 0            │                   │
│                    └───────────────────────────┘                   │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Data Flow

### Sensor Data Flow

```
Pi Hardware
    │
    ├─► GPIO Sensors (Ultrasonic, Line Tracking)
    │
    ▼
sensor_data_publisher (ROS2 Node on Pi)
    │
    ├─► /omega/sensors/ultrasonic
    ├─► /omega/sensors/line_tracking
    └─► /omega/sensors/battery
    │
    ▼
ROS2 DDS Network
    │
    ├─► Lenovo (Native ROS2)
    └─► MacBook Backend (via Web Bridge)
    │
    ▼
Web App (React)
    └─► Real-time visualization
```

### Control Flow

```
Web App (React)
    │
    ├─► User clicks "Forward" button
    │
    ▼
WebSocket: /api/ros/bridge
    │
    ├─► ROS2-Web Bridge Service
    │
    ▼
ROS2 Topic: /cmd_vel (Twist)
    │
    ├─► robot_controller (ROS2 Node)
    │
    ├─► /omega/motors/left
    └─► /omega/motors/right
    │
    ▼
Pi Hardware
    └─► Motor Drivers → Motors
```

## ROS2 Node Graph

```
┌─────────────────────────────────────────────────────────┐
│                  ROS2 Node Graph                        │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  sensor_data_publisher                                  │
│    ├─► /omega/sensors/ultrasonic                        │
│    ├─► /omega/sensors/line_tracking                    │
│    └─► /omega/sensors/battery                           │
│                                                         │
│  robot_controller                                       │
│    ├─► /cmd_vel (subscribes)                            │
│    ├─► /omega/motors/left (publishes)                   │
│    └─► /omega/motors/right (publishes)                  │
│                                                         │
│  enhanced_telemetry                                     │
│    ├─► /omega/sensors/* (subscribes)                   │
│    └─► /omega/telemetry (publishes)                     │
│                                                         │
│  Web Bridge (Backend)                                   │
│    ├─► /omega/sensors/* (subscribes)                   │
│    ├─► /omega/telemetry (subscribes)                    │
│    └─► /cmd_vel (publishes)                             │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Message Types

### Sensor Messages

**Ultrasonic** (`/omega/sensors/ultrasonic`)
```python
Float32
  data: 45.2  # Distance in cm
```

**Line Tracking** (`/omega/sensors/line_tracking`)
```python
Int32MultiArray
  data: [0, 1, 0]  # Left, Center, Right sensor states
```

**Battery** (`/omega/sensors/battery`)
```python
BatteryState
  voltage: 7.4
  percentage: 85.0
  power_supply_status: UNKNOWN
```

### Control Messages

**cmd_vel** (`/cmd_vel`)
```python
Twist
  linear:
    x: 0.5   # Forward speed (-1.0 to 1.0)
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.2   # Rotation speed (-1.0 to 1.0)
```

**Telemetry** (`/omega/telemetry`)
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
    "memory_percent": 45.2
  },
  "status": "active"
}
```

## Web Integration Flow

```
┌──────────────┐
│   Web App    │
│   (React)    │
└──────┬───────┘
       │
       │ WebSocket
       │ ws://host/api/ros/bridge
       ▼
┌──────────────┐
│ ROS2-Web     │
│ Bridge       │
│ (FastAPI)    │
└──────┬───────┘
       │
       │ ROS2 Topics
       │ (rclpy)
       ▼
┌──────────────┐
│  ROS2 DDS    │
│  Network     │
└──────┬───────┘
       │
       ├─► Lenovo Nodes
       └─► Pi Docker Nodes
```

## Deployment Scenarios

### Scenario 1: Development (Lenovo)

```
Lenovo (Native ROS2)
  ├─► sensor_data_publisher (simulated)
  ├─► robot_controller
  ├─► enhanced_telemetry
  └─► Backend with ROS2-Web Bridge
       └─► Web App
```

### Scenario 2: Full System (Lenovo + Pi)

```
Lenovo
  ├─► Backend with ROS2-Web Bridge
  └─► Web App

Pi (Docker ROS2)
  ├─► sensor_data_publisher (real hardware)
  ├─► robot_controller (real motors)
  └─► enhanced_telemetry

Communication: ROS2 DDS (CycloneDDS)
```

### Scenario 3: UI Development (MacBook)

```
MacBook
  ├─► Web App (Next.js)
  └─► Backend (no ROS2)

Pi (Docker ROS2)
  └─► All ROS2 nodes

Communication: Backend controls Pi via SSH
```

## Performance Characteristics

| Component | Latency | Throughput | Notes |
|-----------|---------|------------|-------|
| Sensor Publishing | < 10ms | 10 Hz | Hardware dependent |
| ROS2 DDS | < 5ms | 1000+ msg/s | Network dependent |
| Web Bridge | < 50ms | 100 msg/s | WebSocket overhead |
| Web App Render | < 16ms | 60 FPS | React optimization |

## Security Architecture

```
┌─────────────────────────────────────┐
│         Security Layers              │
├─────────────────────────────────────┤
│                                     │
│  1. Network Isolation                │
│     - ROS_DOMAIN_ID separation      │
│     - Firewall rules                │
│                                     │
│  2. Topic Namespace                  │
│     - /omega/* prefix               │
│     - Prevents topic collision      │
│                                     │
│  3. Command Validation               │
│     - Rate limiting                 │
│     - Value clamping                │
│     - Emergency stop                │
│                                     │
│  4. Authentication (Future)          │
│     - WebSocket auth                │
│     - ROS2 node authentication      │
│                                     │
└─────────────────────────────────────┘
```

## Scalability

### Current Capacity
- **Topics**: 20+ topics supported
- **Subscribers**: Unlimited per topic
- **Publishers**: Multiple per topic
- **Web Clients**: 10+ concurrent connections

### Future Scaling
- **Multi-robot**: Use ROS_DOMAIN_ID per robot
- **Fleet management**: ROS2 namespaces
- **Cloud integration**: ROS2 bridge to cloud
- **Edge computing**: Distribute nodes across devices

---

**Last Updated**: 2024  
**Version**: 1.0

