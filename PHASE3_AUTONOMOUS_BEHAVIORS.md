# Phase 3: Autonomous Behaviors

Complete autonomous behavior system using ROS2 action servers.

## 🎯 Goals

- Implement ROS2 action servers for autonomous behaviors
- Enable web app to trigger and monitor autonomous actions
- Support goal cancellation and feedback
- Integrate with existing sensor and control systems

## ✅ Implemented

### 1. Action Servers

#### Navigate to Goal
**File**: `ros/src/omega_robot/omega_robot/navigate_to_goal_action.py`

**Action**: Navigate to a target position (x, y, theta)

**Features**:
- Uses cmd_vel for movement
- Obstacle detection via ultrasonic sensor
- Real-time feedback on distance remaining
- Automatic goal cancellation on obstacle

**Usage**:
```bash
ros2 run omega_robot navigate_to_goal_action_server
```

#### Follow Line
**File**: `ros/src/omega_robot/omega_robot/follow_line_action.py`

**Action**: Follow a line on the ground using line tracking sensors

**Features**:
- Uses line tracking sensors
- Configurable duration (0 = infinite)
- Line lost detection
- Real-time sensor feedback

**Usage**:
```bash
ros2 run omega_robot follow_line_action_server
```

#### Obstacle Avoidance
**File**: `ros/src/omega_robot/omega_robot/obstacle_avoidance_action.py`

**Action**: Move while avoiding obstacles

**Features**:
- Ultrasonic sensor integration
- Direction control (forward, backward, left, right)
- Automatic obstacle avoidance
- Distance-based target

**Usage**:
```bash
ros2 run omega_robot obstacle_avoidance_action_server
```

### 2. Action Bridge

**File**: `servers/robot-controller-backend/api/ros_action_bridge.py`

**Features**:
- WebSocket-based action goal sending
- Real-time feedback streaming
- Goal cancellation support
- Result handling

### 3. React Component

**File**: `ui/robot-controller-ui/src/components/ros/AutonomousActions.tsx`

**Features**:
- UI for all three actions
- Goal input forms
- Active action monitoring
- Cancel button for active actions
- Real-time status updates

## 📡 Action Topics

### Action Names

| Action | Topic | Description |
|--------|-------|-------------|
| Navigate to Goal | `/navigate_to_goal` | Navigate to (x, y, theta) |
| Follow Line | `/follow_line` | Follow line for duration |
| Obstacle Avoidance | `/obstacle_avoidance` | Move while avoiding obstacles |

### Goal Formats

**Navigate to Goal**:
```json
{
  "x": 1.0,
  "y": 0.0,
  "theta": 0.0
}
```

**Follow Line**:
```json
{
  "duration": 10.0  // seconds, 0 = infinite
}
```

**Obstacle Avoidance**:
```json
{
  "direction": "forward",  // forward, backward, left, right
  "distance": 0.0  // meters, 0 = infinite
}
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
# Launch all action servers
ros2 launch omega_robot robot_full.launch.py

# Or individual servers
ros2 run omega_robot navigate_to_goal_action_server
ros2 run omega_robot follow_line_action_server
ros2 run omega_robot obstacle_avoidance_action_server
```

### 3. Use in Web App

```tsx
import { AutonomousActions } from '@/components/ros';

<AutonomousActions />
```

## 🎮 Web Integration

### Send Action Goal

```javascript
const ws = new WebSocket('ws://localhost:8000/api/ros/bridge');

// Navigate to goal
ws.send(JSON.stringify({
  type: "send_action_goal",
  action: "navigate_to_goal",
  goal: {
    x: 1.0,
    y: 0.0,
    theta: 0.0
  }
}));

// Follow line
ws.send(JSON.stringify({
  type: "send_action_goal",
  action: "follow_line",
  goal: {
    duration: 10.0
  }
}));

// Obstacle avoidance
ws.send(JSON.stringify({
  type: "send_action_goal",
  action: "obstacle_avoidance",
  goal: {
    direction: "forward",
    distance: 0.0
  }
}));
```

### Receive Feedback

```javascript
ws.onmessage = (event) => {
  const msg = JSON.parse(event.data);
  
  if (msg.type === "action_goal_accepted") {
    console.log("Goal accepted:", msg.goal_id);
  } else if (msg.type === "action_feedback") {
    console.log("Feedback:", msg.feedback);
  } else if (msg.type === "action_result") {
    console.log("Result:", msg.result);
  }
};
```

### Cancel Goal

```javascript
ws.send(JSON.stringify({
  type: "cancel_action",
  goal_id: "goal-uuid-here"
}));
```

## 🔧 Testing

### Test from Command Line

```bash
# List actions
ros2 action list

# Get action info
ros2 action info /navigate_to_goal

# Send goal (using action client - requires action definition)
# For now, use web interface or direct cmd_vel
```

### Test Web Integration

```bash
# Start backend
cd servers/robot-controller-backend
python main_api.py

# Open browser and use AutonomousActions component
# Or test via WebSocket console
```

## 📊 Action States

| State | Description |
|-------|-------------|
| `active` | Action is executing |
| `completed` | Action completed successfully |
| `cancelled` | Action was cancelled |
| `error` | Action failed |

## 🐛 Troubleshooting

### Actions Not Available

```bash
# Check action servers are running
ros2 node list | grep action

# Check action topics
ros2 action list
```

### Web Bridge Not Working

```bash
# Check ROS2 is available
export ROS_NATIVE_MODE=true
source ~/.ros2_rolling_setup.bash

# Check backend logs
# Should see "Action bridge initialized"
```

### Action Not Executing

- Verify sensors are publishing (ultrasonic, line tracking)
- Check cmd_vel is being published
- Verify robot_controller node is running
- Check action server logs

## 📈 Next Steps

### Enhancements

- [ ] Proper action type definitions (omega_robot_interfaces)
- [ ] Odometry integration for accurate navigation
- [ ] Path planning integration
- [ ] Mission planning (sequence of actions)
- [ ] Action preemption
- [ ] Action history/logging

### Phase 4 Integration

- [ ] SLAM-based navigation
- [ ] Map-based path planning
- [ ] Waypoint following
- [ ] Autonomous patrol routes

---

**Status**: Phase 3 Complete ✅  
**Next**: Phase 4 - Navigation & SLAM  
**Last Updated**: 2024

