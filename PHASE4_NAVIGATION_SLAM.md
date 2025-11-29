# Phase 4: Navigation & SLAM

Complete navigation and path planning system for Omega Robot.

## 🎯 Goals

- Publish robot odometry for position tracking
- Implement path planning algorithms (A*, D*, RRT)
- Visualize robot position and path in web app
- Support goal-based navigation
- Foundation for SLAM integration

## ✅ Implemented

### 1. Odometry Publisher

**File**: `ros/src/omega_robot/omega_robot/odometry_publisher.py`

**Features**:
- Publishes `/odom` (nav_msgs/Odometry)
- Publishes TF transforms (odom -> base_link)
- Calculates pose from motor speeds
- Differential drive kinematics
- Configurable wheel parameters

**Topics**:
- **Subscribes**: `/omega/motors/left`, `/omega/motors/right`
- **Publishes**: `/odom`, `/tf`

**Parameters**:
- `wheel_radius` (default: 0.05m)
- `wheel_base` (default: 0.20m)
- `ticks_per_revolution` (default: 360)

**Usage**:
```bash
ros2 run omega_robot odometry_publisher
```

### 2. Path Planner

**File**: `ros/src/omega_robot/omega_robot/path_planner.py`

**Features**:
- A* path planning algorithm
- Support for D* and RRT (placeholders)
- Map-based obstacle avoidance
- Goal pose subscription

**Topics**:
- **Subscribes**: `/goal_pose` (geometry_msgs/PoseStamped), `/map` (nav_msgs/OccupancyGrid)
- **Publishes**: `/plan` (nav_msgs/Path)

**Parameters**:
- `algorithm` (default: 'astar') - 'astar', 'dstar', 'rrt'

**Usage**:
```bash
ros2 run omega_robot path_planner
```

### 3. Map Viewer Component

**File**: `ui/robot-controller-ui/src/components/ros/MapViewer.tsx`

**Features**:
- Real-time robot position visualization
- Path display
- Goal setting (click on map)
- Grid overlay
- Coordinate system visualization

**Usage**:
```tsx
import { MapViewer } from '@/components/ros';

<MapViewer width={600} height={600} />
```

## 📡 Topics

### Navigation Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | Odometry | Robot position and velocity |
| `/plan` | Path | Planned path from current to goal |
| `/goal_pose` | PoseStamped | Target position for navigation |
| `/map` | OccupancyGrid | Map for path planning (optional) |
| `/tf` | TFMessage | Transform tree |

### Message Formats

**Odometry** (`/odom`):
```python
Odometry
  header:
    frame_id: "odom"
    stamp: {sec, nanosec}
  pose:
    pose:
      position: {x, y, z}
      orientation: {x, y, z, w}  # quaternion
  twist:
    twist:
      linear: {x, y, z}
      angular: {x, y, z}
```

**Path** (`/plan`):
```python
Path
  header:
    frame_id: "odom"
  poses: [PoseStamped, ...]  # Waypoints
```

## 🚀 Quick Start

### 1. Build

```bash
cd ~/omega_ws
colcon build --packages-select omega_robot
source install/setup.bash
```

### 2. Launch

```bash
# Launch all nodes including navigation
ros2 launch omega_robot robot_full.launch.py

# Or individual nodes
ros2 run omega_robot odometry_publisher
ros2 run omega_robot path_planner
```

### 3. Use in Web App

```tsx
import { MapViewer } from '@/components/ros';

<MapViewer />
```

## 🎮 Web Integration

### Subscribe to Odometry

```javascript
const ws = new WebSocket('ws://localhost:8000/api/ros/bridge');

// Subscribe to odometry
ws.send(JSON.stringify({
  type: "subscribe",
  topic: "/odom",
  msg_type: "Odometry"
}));

// Receive robot pose
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === "ros2_message" && msg.topic === "/odom") {
    const pose = msg.data.pose.pose;
    console.log("Robot position:", pose.position);
  }
};
```

### Send Goal

```javascript
// Send goal pose
ws.send(JSON.stringify({
  type: "publish",
  topic: "/goal_pose",
  msg_type: "PoseStamped",
  command: {
    pose: {
      position: { x: 1.0, y: 0.5, z: 0.0 },
      orientation: { x: 0, y: 0, z: 0, w: 1.0 }
    }
  }
}));
```

## 🔧 Testing

### Test Odometry

```bash
# Run odometry publisher
ros2 run omega_robot odometry_publisher

# In another terminal, echo odometry
ros2 topic echo /odom

# Check transforms
ros2 run tf2_ros tf2_echo odom base_link
```

### Test Path Planning

```bash
# Run path planner
ros2 run omega_robot path_planner

# Send goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}}}"

# View path
ros2 topic echo /plan
```

### Test Web Integration

```bash
# Start backend
cd servers/robot_controller_backend
python main_api.py

# Open browser and use MapViewer component
# Click on map to set goal
```

## 📊 Coordinate Frames

```
odom (fixed frame)
  └── base_link (robot frame)
       └── camera_frame (camera frame)
```

- **odom**: Origin at robot start position
- **base_link**: Robot center
- **camera_frame**: Camera position

## 🐛 Troubleshooting

### Odometry Not Updating

```bash
# Check motor topics are publishing
ros2 topic echo /omega/motors/left
ros2 topic echo /omega/motors/right

# Check odometry node is running
ros2 node list | grep odometry
```

### Path Not Generated

```bash
# Check goal was received
ros2 topic echo /goal_pose

# Check path planner is running
ros2 node list | grep path_planner

# Check path topic
ros2 topic echo /plan
```

### TF Not Publishing

```bash
# Install tf2_ros
sudo apt install ros-rolling-tf2-ros

# Check transforms
ros2 run tf2_ros tf2_echo odom base_link
```

## 📈 Next Steps

### Enhancements

- [ ] Encoder-based odometry (more accurate)
- [ ] IMU integration for heading
- [ ] Proper A* implementation with obstacles
- [ ] D* Lite implementation
- [ ] RRT* implementation
- [ ] Map server integration
- [ ] Costmap generation

### SLAM Integration

- [ ] SLAM node (gmapping/cartographer)
- [ ] Map building
- [ ] Localization (AMCL)
- [ ] Map saving/loading
- [ ] Loop closure detection

### Phase 5 Integration

- [ ] Bag file recording of navigation
- [ ] Navigation diagnostics
- [ ] Multi-robot coordination
- [ ] Mission planning with waypoints

---

**Status**: Phase 4 Complete ✅  
**Next**: Phase 5 - Advanced Features  
**Last Updated**: 2024

