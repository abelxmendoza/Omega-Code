# OMEGA ROBOTICS — FULL DEVELOPMENT BLUEPRINT

**Cursor → GitHub → Docker (ROS2 Humble) → Pi Runtime**

This is the *official* workflow for Omega-Code.

## Directory Structure (Inside Cursor)

```
Omega-Code/
  .devcontainer/
    devcontainer.json
    Dockerfile
  docker/
    ros2_robot/
      Dockerfile.dev
      docker-compose.dev.yml
      entrypoint.dev.sh
      config/cyclonedds.xml
  servers/
    robot_controller_backend/
      video/
        pi_sensor_hub.py
        camera_manager.py
        video_server.py
        motion_detection.py
        ...
  ros/
    launch/
      pi_only.launch.py
      pi_orin_hybrid.launch.py
      omega_full.launch.py
    omega_camera/
    msg/
    srv/
  scripts/
    start_robot.sh
    quick_start_ros2.sh
```

## Official Workflow

**Cursor (MacBook): WRITE THE CODE**  
**GitHub: SYNC THE CODE**  
**Docker (Pi/Jetson): RUN THE CODE**

### Workflow Steps:

1. **In Cursor** → write/refactor/update Omega-Code normally
2. **Commit & push:**
   ```bash
   git add .
   git commit -m "update camera + ROS pipeline"
   git push
   ```
3. **On Pi (host):**
   ```bash
   cd ~/Omega-Code
   git pull
   ```
4. **Rebuild + run Dev container:**
   ```bash
   cd docker/ros2_robot
   docker compose -f docker-compose.dev.yml up --build
   ```

## .devcontainer Config (Cursor uses to give ROS intelli-sense)

**File:** `.devcontainer/devcontainer.json`

Provides ROS2 Humble intellisense and development environment inside Cursor/VSCode.

## .devcontainer Dockerfile

**File:** `.devcontainer/Dockerfile`

Base ROS2 Humble image with:
- `python3-opencv`
- `python3-picamera2`
- `libcamera-apps`
- ROS2 packages (`cv_bridge`, `image_transport`, etc.)

## ROS2 Dev Container (For the Pi)

**File:** `docker/ros2_robot/Dockerfile.dev`

Runtime container with:
- ROS2 Humble
- Pi camera support (`picamera2`, `libcamera-apps`)
- OpenCV
- All ROS2 message types

## Dev Compose File

**File:** `docker/ros2_robot/docker-compose.dev.yml`

- Runs `pi_sensor_hub.py` automatically
- Mounts `/dev:/dev` for camera access
- Mounts Omega-Code repo for hot-swapping
- Uses `privileged` mode for hardware access

## Docker Entrypoint

**File:** `docker/ros2_robot/entrypoint.dev.sh`

Simple script that:
- Sources ROS2 environment
- Sets `PYTHONUNBUFFERED=1` for better logging
- Executes the command passed to container

## Running the Stack

**On the Pi:**

```bash
cd ~/Omega-Code/docker/ros2_robot
docker compose -f docker-compose.dev.yml down
docker compose -f docker-compose.dev.yml up --build
```

**Check ROS topics:**

```bash
docker exec -it omega_ros2 bash
source /opt/ros/humble/setup.bash
ros2 topic list
```

## Purpose of Each Layer

### Cursor:
- Full code editing
- Refactors
- LLM-powered engineering
- Repo-wide analysis
- Docs + comments
- Architecture changes

### Docker (runtime):
- ROS2 Humble
- Pi camera support (Picamera2/libcamera)
- OpenCV
- rclpy
- Hybrid Pi → Orin brain link
- REST/WebSocket server runtime
- DDS communication

### Pi:
- Actual sensors
- Real cameras
- GPIO / servos
- Network bridge

## Best Practices

✅ **Always code in Cursor**  
✅ **Always run ROS inside Docker**  
✅ **Never modify files inside the container (use repo volume)**  
✅ **Always push → pull before running**  
✅ **Keep Dockerfile.dev lean & focused**

## Quick Reference

### Development Cycle:
1. Edit code in Cursor
2. `git commit && git push`
3. On Pi: `git pull`
4. `docker compose -f docker-compose.dev.yml up --build`

### Debugging:
```bash
# Enter container
docker exec -it omega_ros2 bash

# Check ROS topics
ros2 topic list
ros2 topic echo /omega/camera/compressed

# Check container logs
docker logs omega_ros2

# Rebuild workspace
cd /root/omega_ws
colcon build --symlink-install
source install/setup.bash
```

---

**END OF BLUEPRINT — THIS IS YOUR OFFICIAL SETUP**

