# Omega-Code Repository File Summary

This document provides a comprehensive summary of every file in the Omega-Code repository, including purpose, key functions, side effects, external dependencies, and architectural patterns.

## Table of Contents

- [Root Level Files](#root-level-files)
- [Backend Server Files](#backend-server-files)
- [Frontend UI Files](#frontend-ui-files)
- [ROS2 Files](#ros2-files)
- [Scripts](#scripts)
- [Docker Configuration](#docker-configuration)
- [Documentation](#documentation)

---

## Root Level Files

### Configuration Files

#### `Makefile`
- **Purpose**: Build automation and development shortcuts
- **Key Functions**: 
  - `make check` - Profile-aware endpoint checking
  - `make ui-dev` - Start Next.js dev server
  - `make run-movement` - Start movement WebSocket server
  - `make backend-install` - Install Python dependencies
- **Side Effects**: Loads environment variables from `.env` files
- **Dependencies**: bash, make, npm, python3
- **Patterns**: Environment variable loading, profile-based configuration

#### `docker-compose.yml`
- **Purpose**: Multi-container orchestration for full stack
- **Key Services**: frontend, backend, redis, nginx
- **Side Effects**: Creates Docker network `robot-network`, volumes for Redis
- **Dependencies**: Docker, docker-compose
- **Patterns**: Service-oriented architecture, reverse proxy pattern

#### `deploy.sh`
- **Purpose**: Deployment automation script
- **Key Functions**: Builds Docker images, starts services, health checks
- **Side Effects**: Creates directories (logs, ssl, data), starts containers
- **Dependencies**: docker, docker-compose, curl
- **Patterns**: Infrastructure as code, health check pattern

#### `vercel.json`
- **Purpose**: Vercel deployment configuration
- **Key Functions**: Build commands, output directory, framework detection
- **Side Effects**: Configures Next.js build for Vercel
- **Dependencies**: Vercel platform
- **Patterns**: Platform-as-a-service configuration

#### `nginx.conf`
- **Purpose**: Reverse proxy and load balancing configuration
- **Key Functions**: Routes frontend/backend, WebSocket proxying, rate limiting
- **Side Effects**: Configures upstream servers, caching headers
- **Dependencies**: nginx
- **Patterns**: Reverse proxy pattern, rate limiting, WebSocket upgrade handling

#### `cursor.config.json`
- **Purpose**: Cursor IDE project configuration
- **Key Functions**: Defines project structure, technologies, conventions
- **Side Effects**: Provides IDE hints and project context
- **Dependencies**: Cursor IDE
- **Patterns**: Project metadata, technology stack documentation

### Python Analysis Scripts

#### `hardware_capability_analysis.py`
- **Purpose**: Analyze hardware capabilities for robot components
- **Key Functions**: Hardware detection, capability assessment
- **Side Effects**: Generates analysis reports
- **Dependencies**: Python 3, hardware detection libraries
- **Patterns**: Hardware abstraction, capability detection

#### `pi4b_8gb_algorithms.py`
- **Purpose**: Algorithm analysis for Raspberry Pi 4B 8GB
- **Key Functions**: Performance analysis, algorithm benchmarking
- **Side Effects**: Generates performance metrics
- **Dependencies**: Python 3, performance libraries
- **Patterns**: Performance analysis, benchmarking

#### `pi4b_realistic_analysis.py`, `pi4b_realistic_only.py`
- **Purpose**: Realistic performance analysis for Pi 4B
- **Key Functions**: Hardware-specific performance modeling
- **Side Effects**: Generates realistic performance estimates
- **Dependencies**: Python 3
- **Patterns**: Performance modeling, hardware-specific optimization

#### `update_servo_center.py`
- **Purpose**: Update servo center positions
- **Key Functions**: Servo calibration, center position adjustment
- **Side Effects**: Modifies servo configuration
- **Dependencies**: Python 3, servo control libraries
- **Patterns**: Hardware calibration, configuration management

---

## Backend Server Files

### Main Entry Points

#### `servers/robot-controller-backend/main_api.py`
- **Purpose**: FastAPI application entry point
- **Key Functions**: 
  - Creates FastAPI app with CORS middleware
  - Includes API routers (lighting, autonomy, ROS, capability)
  - Runs uvicorn server
- **Side Effects**: Starts HTTP server on port 8000
- **Dependencies**: fastapi, uvicorn, api module
- **Patterns**: API gateway pattern, modular routing

#### `servers/robot-controller-backend/main.go`
- **Purpose**: Go WebSocket server entry point
- **Key Functions**: Calls `core.StartServer()`
- **Side Effects**: Starts HTTP/WebSocket server
- **Dependencies**: Go 1.22+, core package
- **Patterns**: Simple entry point pattern

#### `servers/robot-controller-backend/main_combined.go`
- **Purpose**: Combined Go server with multiple services
- **Key Functions**: Unified server for multiple WebSocket endpoints
- **Side Effects**: Starts multiple services in one process
- **Dependencies**: Go 1.22+, core package
- **Patterns**: Monolithic service pattern

#### `servers/robot-controller-backend/robot_controller_server.py`
- **Purpose**: Python robot controller server
- **Key Functions**: Main robot control logic
- **Side Effects**: Manages robot state and commands
- **Dependencies**: Python 3.11+, asyncio, websockets
- **Patterns**: Server pattern, state management

### API Layer (`api/`)

#### `api/__init__.py`
- **Purpose**: FastAPI router aggregation
- **Key Functions**: Combines lighting, autonomy, ROS, capability routers
- **Side Effects**: Exposes all API routes under `/api`
- **Dependencies**: fastapi, route modules
- **Patterns**: Router composition pattern

#### `api/lighting_routes.py`
- **Purpose**: REST API endpoints for LED lighting control
- **Key Functions**: 
  - `POST /lighting/set` - Set LED colors
  - `POST /lighting/pattern` - Set lighting patterns
  - `GET /lighting/status` - Get lighting status
- **Side Effects**: Controls physical LED hardware
- **Dependencies**: fastapi, lighting controllers
- **Patterns**: REST API pattern, hardware abstraction

#### `api/autonomy_routes.py`
- **Purpose**: REST API for autonomous behavior control
- **Key Functions**: 
  - `POST /autonomy/start` - Start autonomy mode
  - `POST /autonomy/stop` - Stop autonomy
  - `POST /autonomy/update` - Update parameters
- **Side Effects**: Controls robot autonomous behavior
- **Dependencies**: fastapi, autonomy controller
- **Patterns**: REST API pattern, state machine pattern

#### `api/ros_routes.py`
- **Purpose**: ROS2 integration REST endpoints
- **Key Functions**: Bridge between web API and ROS2 topics
- **Side Effects**: Publishes/subscribes to ROS2 topics
- **Dependencies**: fastapi, rclpy (ROS2)
- **Patterns**: Bridge pattern, pub/sub pattern

#### `api/ros_web_bridge.py`
- **Purpose**: WebSocket bridge for ROS2 communication
- **Key Functions**: Bidirectional ROS2-WebSocket communication
- **Side Effects**: Maintains WebSocket connections, ROS2 subscriptions
- **Dependencies**: websockets, rclpy
- **Patterns**: Bridge pattern, WebSocket pattern

#### `api/ros_native_bridge.py`
- **Purpose**: Native ROS2 integration (no Docker)
- **Key Functions**: Direct rclpy integration for Lenovo development
- **Side Effects**: Requires ROS2 native installation
- **Dependencies**: rclpy, ROS2 Rolling/Humble
- **Patterns**: Native integration pattern

#### `api/ros_action_bridge.py`
- **Purpose**: ROS2 action server bridge
- **Key Functions**: Exposes ROS2 actions via REST API
- **Side Effects**: Creates ROS2 action servers/clients
- **Dependencies**: rclpy, actionlib
- **Patterns**: Action pattern, bridge pattern

#### `api/capability_routes.py`
- **Purpose**: System capability detection API
- **Key Functions**: Reports hardware/software capabilities
- **Side Effects**: Queries system capabilities
- **Dependencies**: fastapi, capability service
- **Patterns**: Capability pattern, feature detection

#### `api/capability_service.py`
- **Purpose**: Capability detection service
- **Key Functions**: Detects hardware capabilities, camera resolutions, ROS2 availability
- **Side Effects**: Queries system hardware/software
- **Dependencies**: Python 3, hardware detection libraries
- **Patterns**: Service pattern, capability detection

#### `api/performance_api.py`
- **Purpose**: Performance metrics API
- **Key Functions**: Exposes performance monitoring endpoints
- **Side Effects**: Collects and serves performance metrics
- **Dependencies**: fastapi, performance monitoring
- **Patterns**: Metrics pattern, monitoring pattern

#### `api/sensor_routes.py`
- **Purpose**: Sensor data REST API
- **Key Functions**: Exposes sensor readings via REST
- **Side Effects**: Reads sensor hardware
- **Dependencies**: fastapi, sensor drivers
- **Patterns**: REST API pattern, sensor abstraction

### Movement Control (`movement/`)

#### `movement/movement_ws_server.py`
- **Purpose**: WebSocket server for robot movement control
- **Key Functions**: 
  - Handles movement commands (forward, backward, left, right, stop)
  - Servo control (horizontal/vertical)
  - Buzzer control (buzz, buzz-for, buzz-pulse)
  - Speed control
  - Autonomy hand-off
  - Straight-drive assist
- **Side Effects**: Controls motors, servos, buzzer hardware
- **Dependencies**: websockets, motor controllers, servo controllers, buzzer
- **Patterns**: WebSocket server pattern, command pattern, state management
- **Architecture**: 
  - Async/await for non-blocking I/O
  - Motor lock for thread safety
  - Client tracking for safe shutdown
  - Message batching optimization
  - Caching for telemetry

#### `movement/simple_movement_ws_server.py`
- **Purpose**: Simplified movement WebSocket server
- **Key Functions**: Basic movement commands without advanced features
- **Side Effects**: Controls motors
- **Dependencies**: websockets, motor controllers
- **Patterns**: Simplified server pattern

#### `movement/motor_telemetry.py`
- **Purpose**: Motor telemetry collection and reporting
- **Key Functions**: Collects motor speed, current, temperature data
- **Side Effects**: Reads motor sensors
- **Dependencies**: Motor hardware interfaces
- **Patterns**: Telemetry pattern, sensor reading

#### `movement/straight_drive_assist.py`
- **Purpose**: Assists robot in driving straight
- **Key Functions**: Trim adjustment, drift correction
- **Side Effects**: Adjusts motor speeds for straight driving
- **Dependencies**: Motor controllers
- **Patterns**: Control algorithm pattern, PID-like correction

#### `movement/PCA9685.py`
- **Purpose**: PCA9685 PWM controller interface
- **Key Functions**: Controls PWM signals for servos/motors
- **Side Effects**: Controls I2C PWM hardware
- **Dependencies**: I2C libraries, PCA9685 hardware
- **Patterns**: Hardware abstraction pattern

#### `movement/movement.go`
- **Purpose**: Go movement control implementation
- **Key Functions**: Motor control via GPIO
- **Side Effects**: Controls GPIO pins for motors
- **Dependencies**: Go GPIO libraries
- **Patterns**: Hardware abstraction pattern

### Video System (`video/`)

#### `video/video_server.py`
- **Purpose**: Flask-based MJPEG video streaming server
- **Key Functions**: 
  - `/video_feed` - Main MJPEG stream
  - `/video_feed_low`, `/video_feed_medium`, `/video_feed_high` - Multi-resolution streams
  - `/health` - Health check with camera status
  - `/snapshot` - Single frame capture
  - `/metrics` - Performance metrics
  - `/camera/list` - List available cameras
  - `/camera/switch` - Switch camera device
  - `/recording/start`, `/recording/stop`, `/recording/status` - Video recording
  - `/overlays/config` - Configure frame overlays
- **Side Effects**: 
  - Captures video frames from camera
  - Applies motion detection, face recognition, ArUco detection
  - Records video to disk
  - Publishes frames to ROS2 (optional)
- **Dependencies**: flask, opencv-python, picamera2, camera module
- **Patterns**: 
  - MJPEG streaming pattern
  - Hardware-aware optimization (Pi 4B vs Jetson)
  - Watchdog pattern for camera recovery
  - Multi-resolution streaming
  - Frame overlay pattern
- **Architecture**:
  - Hardware detection for optimization
  - Adaptive JPEG quality based on CPU load
  - Frame skipping under heavy load
  - Background camera initialization retries
  - Stall detection and recovery

#### `video/camera.py`
- **Purpose**: Unified camera abstraction layer
- **Key Functions**: 
  - Auto-detects Picamera2 vs V4L2
  - Background frame capture thread
  - Frame retrieval with latest frame caching
  - Hardware detection for optimization
- **Side Effects**: Opens camera device, captures frames
- **Dependencies**: picamera2, opencv-python, numpy
- **Patterns**: 
  - Strategy pattern (Picamera2 vs V4L2)
  - Thread-safe frame access
  - Hardware abstraction pattern
- **Architecture**: Background thread captures frames, main thread reads latest

#### `video/motion_detection.py`
- **Purpose**: Motion detection in video frames
- **Key Functions**: Detects motion, draws bounding boxes
- **Side Effects**: Processes frames, modifies frame data
- **Dependencies**: opencv-python, numpy
- **Patterns**: Computer vision pattern, background subtraction

#### `video/face_recognition.py`
- **Purpose**: Face recognition and detection
- **Key Functions**: Detects and recognizes faces in frames
- **Side Effects**: Loads face encodings, processes frames
- **Dependencies**: face-recognition, dlib, opencv-python
- **Patterns**: Computer vision pattern, ML pattern

#### `video/aruco_detection.py`
- **Purpose**: ArUco marker detection
- **Key Functions**: Detects ArUco markers, estimates pose
- **Side Effects**: Processes frames, detects markers
- **Dependencies**: opencv-contrib-python
- **Patterns**: Computer vision pattern, marker detection

#### `video/object_tracking.py`
- **Purpose**: Object tracking in video stream
- **Key Functions**: Tracks selected objects across frames
- **Side Effects**: Maintains tracking state, modifies frames
- **Dependencies**: opencv-python
- **Patterns**: Computer vision pattern, tracking algorithm

#### `video/video_recorder.py`
- **Purpose**: Video recording functionality
- **Key Functions**: Records video frames to file
- **Side Effects**: Writes video files to disk
- **Dependencies**: opencv-python
- **Patterns**: Recording pattern, file I/O pattern

#### `video/frame_overlays.py`
- **Purpose**: Frame overlay rendering
- **Key Functions**: Adds timestamp, FPS, telemetry overlays
- **Side Effects**: Modifies frame data
- **Dependencies**: opencv-python
- **Patterns**: Overlay pattern, frame processing

#### `video/frame_buffer.py`
- **Purpose**: Frame buffering for replay
- **Key Functions**: Buffers recent frames, provides replay
- **Side Effects**: Stores frames in memory
- **Dependencies**: numpy, threading
- **Patterns**: Buffer pattern, circular buffer

#### `video/ros2_integration.py`
- **Purpose**: ROS2 video frame publishing
- **Key Functions**: Publishes frames to ROS2 topics
- **Side Effects**: Creates ROS2 publishers, publishes messages
- **Dependencies**: rclpy, sensor_msgs
- **Patterns**: ROS2 integration pattern, pub/sub pattern

#### `video/websocket_server.py`
- **Purpose**: WebSocket server for video metrics
- **Key Functions**: Broadcasts video metrics to WebSocket clients
- **Side Effects**: Maintains WebSocket connections
- **Dependencies**: flask-socketio (optional)
- **Patterns**: WebSocket pattern, metrics broadcasting

#### `video/mock_camera_server.py`
- **Purpose**: Mock camera for development/testing
- **Key Functions**: Generates test frames without hardware
- **Side Effects**: Creates synthetic frames
- **Dependencies**: numpy, opencv-python
- **Patterns**: Mock pattern, test pattern

### Sensors (`sensors/`)

#### `sensors/main_ultrasonic.go`
- **Purpose**: Go WebSocket server for ultrasonic sensor
- **Key Functions**: Reads HC-SR04 ultrasonic sensor, streams distance data
- **Side Effects**: Controls GPIO pins, reads sensor
- **Dependencies**: Go GPIO libraries, gorilla/websocket
- **Patterns**: WebSocket server pattern, sensor reading pattern

#### `sensors/ultrasonic_ws_server.py`
- **Purpose**: Python WebSocket server for ultrasonic sensor
- **Key Functions**: Alternative Python implementation
- **Side Effects**: Reads ultrasonic sensor
- **Dependencies**: websockets, GPIO libraries
- **Patterns**: WebSocket server pattern

#### `sensors/ultrasonic_sensor.py`
- **Purpose**: Ultrasonic sensor driver
- **Key Functions**: Low-level sensor reading
- **Side Effects**: Controls GPIO pins
- **Dependencies**: GPIO libraries
- **Patterns**: Hardware driver pattern

#### `sensors/line_tracking_ws_server.py`
- **Purpose**: Line tracking sensor WebSocket server
- **Key Functions**: Reads IR line tracking sensors, streams state
- **Side Effects**: Reads GPIO pins
- **Dependencies**: websockets, GPIO libraries
- **Patterns**: WebSocket server pattern, sensor reading

### Controllers (`controllers/`)

#### `controllers/servo_control.py`
- **Purpose**: Servo motor control
- **Key Functions**: Sets servo angles (horizontal/vertical)
- **Side Effects**: Controls servo hardware
- **Dependencies**: PCA9685 or GPIO libraries
- **Patterns**: Hardware abstraction pattern

#### `controllers/buzzer.py`
- **Purpose**: Buzzer control
- **Key Functions**: Turns buzzer on/off
- **Side Effects**: Controls buzzer GPIO pin
- **Dependencies**: GPIO libraries
- **Patterns**: Hardware abstraction pattern

#### `controllers/line_tracking.py`
- **Purpose**: Line tracking controller
- **Key Functions**: Processes line sensor data
- **Side Effects**: Reads line sensors
- **Dependencies**: GPIO libraries
- **Patterns**: Control algorithm pattern

#### `controllers/advanced_pid.py`
- **Purpose**: Advanced PID controller
- **Key Functions**: PID control algorithms
- **Side Effects**: Computes control outputs
- **Dependencies**: numpy, scipy
- **Patterns**: Control algorithm pattern, PID pattern

#### `controllers/lighting/led_control.py`
- **Purpose**: LED strip control (WS2812/WS2811)
- **Key Functions**: Sets LED colors, patterns
- **Side Effects**: Controls LED hardware via SPI/GPIO
- **Dependencies**: adafruit-circuitpython-neopixel
- **Patterns**: Hardware abstraction pattern

#### `controllers/lighting/patterns.py`
- **Purpose**: LED pattern implementations
- **Key Functions**: Rainbow, fade, blink, chase, music visualizer patterns
- **Side Effects**: Updates LED colors
- **Dependencies**: led_control module
- **Patterns**: Strategy pattern, pattern library

#### `controllers/lighting/dispatcher.py`
- **Purpose**: Routes lighting commands to patterns
- **Key Functions**: Command routing, pattern selection
- **Side Effects**: Executes lighting patterns
- **Dependencies**: patterns module, led_control
- **Patterns**: Dispatcher pattern, command routing

#### `controllers/lighting/main_lighting.go`
- **Purpose**: Go WebSocket server for lighting control
- **Key Functions**: WebSocket endpoint for LED control
- **Side Effects**: Executes Python LED controller via shell script
- **Dependencies**: Go websocket, shell script wrapper
- **Patterns**: WebSocket server pattern, process execution pattern

### Autonomy (`autonomy/`)

#### `autonomy/base.py`
- **Purpose**: Base class for autonomy mode handlers
- **Key Functions**: Defines autonomy mode interface
- **Side Effects**: None (abstract base)
- **Dependencies**: Python 3, logging
- **Patterns**: Strategy pattern, template method pattern

#### `autonomy/controller.py`
- **Purpose**: Autonomy controller orchestrator
- **Key Functions**: Manages autonomy modes, state transitions
- **Side Effects**: Controls robot autonomous behavior
- **Dependencies**: autonomy modes
- **Patterns**: Controller pattern, state machine pattern

#### `autonomy/modes/simple.py`
- **Purpose**: Simple autonomy mode implementations
- **Key Functions**: Basic autonomous behaviors
- **Side Effects**: Controls robot movement
- **Dependencies**: motor controllers, sensors
- **Patterns**: Strategy pattern implementation

#### `autonomy/modes/ros2_modes.py`
- **Purpose**: ROS2-based autonomy modes
- **Key Functions**: ROS2 action-based autonomy
- **Side Effects**: Publishes/subscribes to ROS2 topics
- **Dependencies**: rclpy, ROS2 actions
- **Patterns**: ROS2 integration pattern, action pattern

### Hardware Abstraction (`hardware/`, `gpio/`)

#### `gpio/gpio_real.go`
- **Purpose**: Real GPIO implementation for Raspberry Pi
- **Key Functions**: GPIO pin control, PWM, interrupts
- **Side Effects**: Controls physical GPIO pins
- **Dependencies**: go-rpio library
- **Patterns**: Hardware abstraction pattern

#### `gpio/gpio_mock.go`
- **Purpose**: Mock GPIO for development/testing
- **Key Functions**: Simulates GPIO without hardware
- **Side Effects**: None (simulation only)
- **Dependencies**: None
- **Patterns**: Mock pattern, test pattern

#### `gpio/motor.go`
- **Purpose**: Motor control via GPIO
- **Key Functions**: Motor forward, backward, stop
- **Side Effects**: Controls motor GPIO pins
- **Dependencies**: GPIO interface
- **Patterns**: Hardware abstraction pattern

#### `hardware/hardware_manager.py`
- **Purpose**: Hardware resource manager
- **Key Functions**: Manages hardware initialization, cleanup
- **Side Effects**: Initializes hardware resources
- **Dependencies**: Hardware drivers
- **Patterns**: Resource manager pattern

#### `hardware/motor_control.py`
- **Purpose**: Motor control abstraction
- **Key Functions**: High-level motor control interface
- **Side Effects**: Controls motors
- **Dependencies**: GPIO or PCA9685
- **Patterns**: Hardware abstraction pattern

#### `hardware/sensor_drivers.py`
- **Purpose**: Sensor driver implementations
- **Key Functions**: Unified sensor interface
- **Side Effects**: Reads sensors
- **Dependencies**: Sensor-specific libraries
- **Patterns**: Driver pattern, hardware abstraction

#### `hardware/sensor_fusion.py`
- **Purpose**: Multi-sensor data fusion
- **Key Functions**: Combines sensor readings, Kalman filtering
- **Side Effects**: Processes sensor data
- **Dependencies**: numpy, scipy
- **Patterns**: Sensor fusion pattern, filtering pattern

#### `hardware/camera_drivers.py`
- **Purpose**: Camera driver abstraction
- **Key Functions**: Unified camera interface
- **Side Effects**: Controls camera hardware
- **Dependencies**: picamera2, opencv-python
- **Patterns**: Hardware abstraction pattern

### AI/ML (`ai/`)

#### `ai/computer_vision.py`
- **Purpose**: Computer vision processing
- **Key Functions**: Image processing, object detection
- **Side Effects**: Processes images
- **Dependencies**: opencv-python, numpy
- **Patterns**: Computer vision pattern, ML pattern

#### `ai/navigation_system.py`
- **Purpose**: Navigation and path planning
- **Key Functions**: Path planning, obstacle avoidance
- **Side Effects**: Computes navigation paths
- **Dependencies**: numpy, scipy, path planning libraries
- **Patterns**: Navigation pattern, path planning pattern

#### `ai/predictive_analytics.py`
- **Purpose**: Predictive analytics for robot behavior
- **Key Functions**: Behavior prediction, maintenance prediction
- **Side Effects**: Analyzes historical data
- **Dependencies**: scikit-learn, numpy
- **Patterns**: ML pattern, analytics pattern

#### `ai/autonomous_engine.py`
- **Purpose**: Autonomous behavior engine
- **Key Functions**: Decision making, behavior selection
- **Side Effects**: Controls robot behavior
- **Dependencies**: Navigation, computer vision modules
- **Patterns**: Decision engine pattern, behavior tree pattern

### Core (`core/`)

#### `core/core.go`
- **Purpose**: Core WebSocket server functionality
- **Key Functions**: 
  - `StartServer()` - Starts HTTP/WebSocket server
  - `HandleConnections()` - WebSocket connection handler
  - `HandleWebSocketUltrasonicSensor()` - Ultrasonic sensor streaming
- **Side Effects**: Starts server on port 8080, manages WebSocket connections
- **Dependencies**: gorilla/websocket, GPIO libraries
- **Patterns**: WebSocket server pattern, connection handling pattern

### Commands (`commands/`)

#### `commands/commands.go`
- **Purpose**: Command definitions
- **Key Functions**: Command type definitions
- **Side Effects**: None
- **Dependencies**: Go standard library
- **Patterns**: Command pattern

#### `commands/command_processor.go`
- **Purpose**: Command processing logic
- **Key Functions**: Processes robot commands
- **Side Effects**: Executes commands on hardware
- **Dependencies**: Hardware controllers
- **Patterns**: Command processor pattern

#### `commands/motor_control.go`
- **Purpose**: Motor control commands
- **Key Functions**: Motor command execution
- **Side Effects**: Controls motors
- **Dependencies**: Motor hardware
- **Patterns**: Command pattern implementation

#### `commands/servo_control.go`
- **Purpose**: Servo control commands
- **Key Functions**: Servo command execution
- **Side Effects**: Controls servos
- **Dependencies**: Servo hardware
- **Patterns**: Command pattern implementation

#### `commands/ros_integration.go`
- **Purpose**: ROS integration commands
- **Key Functions**: ROS command execution, ROS node management
- **Side Effects**: Starts/stops ROS nodes
- **Dependencies**: ROS system
- **Patterns**: Integration pattern, command pattern

### Servers (`servers/`)

#### `servers/gateway_api.py`
- **Purpose**: Gateway API aggregating multiple services
- **Key Functions**: 
  - Proxies WebSocket connections (`/ws/*`)
  - Video feed proxy (`/video_feed`)
  - Network API (`/api/net/*`)
- **Side Effects**: Routes requests to downstream services
- **Dependencies**: fastapi, httpx, websockets
- **Patterns**: API gateway pattern, reverse proxy pattern

#### `servers/control_api.py`
- **Purpose**: Control API endpoints
- **Key Functions**: Unified control interface
- **Side Effects**: Routes control commands
- **Dependencies**: fastapi
- **Patterns**: API gateway pattern

### Utilities (`utils/`)

#### `utils/env.py`
- **Purpose**: Environment variable utilities
- **Key Functions**: Environment variable loading, validation
- **Side Effects**: Loads `.env` files
- **Dependencies**: python-dotenv
- **Patterns**: Configuration pattern

#### `utils/optimization/websocket_optimizer.py`
- **Purpose**: WebSocket message optimization
- **Key Functions**: Message batching, compression
- **Side Effects**: Optimizes WebSocket traffic
- **Dependencies**: websockets
- **Patterns**: Optimization pattern, batching pattern

#### `utils/optimization/cache_manager.py`
- **Purpose**: Caching system
- **Key Functions**: Redis caching with memory fallback
- **Side Effects**: Caches data, manages cache lifecycle
- **Dependencies**: redis (optional)
- **Patterns**: Cache pattern, strategy pattern

#### `utils/optimization/async_processor.py`
- **Purpose**: Async task processing
- **Key Functions**: Priority queues, task execution
- **Side Effects**: Executes background tasks
- **Dependencies**: asyncio
- **Patterns**: Async pattern, queue pattern

#### `utils/optimization/performance_monitor.py`
- **Purpose**: Performance monitoring
- **Key Functions**: CPU, memory, network monitoring
- **Side Effects**: Collects performance metrics
- **Dependencies**: psutil
- **Patterns**: Monitoring pattern, metrics pattern

### Tests (`tests/`)

#### `tests/unit/`
- **Purpose**: Unit tests for backend components
- **Key Functions**: Tests individual components in isolation
- **Side Effects**: None (test only)
- **Dependencies**: pytest, unittest
- **Patterns**: Unit testing pattern

#### `tests/integration/`
- **Purpose**: Integration tests
- **Key Functions**: Tests component interactions
- **Side Effects**: May start test services
- **Dependencies**: pytest, test services
- **Patterns**: Integration testing pattern

#### `tests/e2e/`
- **Purpose**: End-to-end tests
- **Key Functions**: Tests full system workflows
- **Side Effects**: May start full system
- **Dependencies**: pytest, full system
- **Patterns**: E2E testing pattern

---

## Frontend UI Files

### Configuration

#### `ui/robot-controller-ui/package.json`
- **Purpose**: Node.js project configuration
- **Key Functions**: Defines dependencies, scripts, Jest/Cypress config
- **Side Effects**: Installs npm packages
- **Dependencies**: npm, Node.js 18+
- **Patterns**: Package management pattern

#### `ui/robot-controller-ui/next.config.mjs`
- **Purpose**: Next.js configuration
- **Key Functions**: Build configuration, environment variables
- **Side Effects**: Configures Next.js build
- **Dependencies**: Next.js
- **Patterns**: Framework configuration pattern

#### `ui/robot-controller-ui/tsconfig.json`
- **Purpose**: TypeScript configuration
- **Key Functions**: TypeScript compiler options, paths
- **Side Effects**: Configures TypeScript compilation
- **Dependencies**: TypeScript
- **Patterns**: TypeScript configuration pattern

#### `ui/robot-controller-ui/tailwind.config.ts`
- **Purpose**: Tailwind CSS configuration
- **Key Functions**: Theme customization, plugin configuration
- **Side Effects**: Configures Tailwind styles
- **Dependencies**: Tailwind CSS
- **Patterns**: CSS framework configuration

### Pages (`src/pages/`)

#### `src/pages/index.tsx`
- **Purpose**: Main robot control dashboard
- **Key Functions**: 
  - WebSocket connection management
  - Camera feed display
  - Robot controls (movement, servos, speed)
  - Sensor dashboard
  - Autonomy panel
  - Performance dashboard
- **Side Effects**: 
  - Connects to WebSocket servers
  - Sends robot commands
  - Displays real-time data
- **Dependencies**: React, Next.js, WebSocket client
- **Patterns**: 
  - Component composition pattern
  - WebSocket pattern
  - State management pattern
  - Profile-based configuration
- **Architecture**:
  - Client-side only camera (avoids SSR)
  - Exponential backoff reconnection
  - WebSocket keep-alive
  - Mock mode for development

#### `src/pages/ros.tsx`
- **Purpose**: ROS2 management page
- **Key Functions**: ROS2 topic viewer, node management
- **Side Effects**: Connects to ROS2 bridge
- **Dependencies**: ROS2 WebSocket bridge
- **Patterns**: ROS2 integration pattern

#### `src/pages/network.tsx`
- **Purpose**: Network configuration page
- **Key Functions**: Network profile selection, connection testing
- **Side Effects**: Changes network configuration
- **Dependencies**: Network utilities
- **Patterns**: Configuration pattern

#### `src/pages/analytics.tsx`
- **Purpose**: Analytics dashboard
- **Key Functions**: Performance metrics visualization
- **Side Effects**: Displays analytics data
- **Dependencies**: Analytics APIs
- **Patterns**: Dashboard pattern, data visualization

#### `src/pages/demo.tsx`
- **Purpose**: Demo page
- **Key Functions**: Showcase features
- **Side Effects**: None
- **Dependencies**: Demo components
- **Patterns**: Demo pattern

#### `src/pages/cyber-demo.tsx`
- **Purpose**: Cyber-themed demo
- **Key Functions**: Visual demo with cyber theme
- **Side Effects**: None
- **Dependencies**: Demo components
- **Patterns**: Demo pattern, theming pattern

#### `src/pages/_app.tsx`
- **Purpose**: Next.js app wrapper
- **Key Functions**: Global app configuration, providers
- **Side Effects**: Wraps all pages
- **Dependencies**: Next.js, Redux, contexts
- **Patterns**: App wrapper pattern, provider pattern

#### `src/pages/_document.tsx`
- **Purpose**: Next.js document customization
- **Key Functions**: HTML document structure
- **Side Effects**: Customizes HTML output
- **Dependencies**: Next.js
- **Patterns**: Document customization pattern

#### `src/pages/_error.tsx`, `src/pages/404.tsx`, `src/pages/500.tsx`
- **Purpose**: Error pages
- **Key Functions**: Error handling UI
- **Side Effects**: Displays error pages
- **Dependencies**: Next.js
- **Patterns**: Error handling pattern

### Components (`src/components/`)

#### `src/components/Header.tsx`
- **Purpose**: Application header
- **Key Functions**: Navigation, service status indicators
- **Side Effects**: Displays header
- **Dependencies**: Service status APIs
- **Patterns**: Header component pattern

#### `src/components/CameraFrame.tsx`
- **Purpose**: Camera feed display component
- **Key Functions**: Displays MJPEG stream, health monitoring
- **Side Effects**: Loads video stream
- **Dependencies**: Video proxy API
- **Patterns**: Video component pattern, health monitoring

#### `src/components/VideoFeed.tsx`
- **Purpose**: Video feed component (alternative)
- **Key Functions**: Video stream display
- **Side Effects**: Loads video
- **Dependencies**: Video API
- **Patterns**: Video component pattern

#### `src/components/CarControlPanel.tsx`
- **Purpose**: Robot movement control panel
- **Key Functions**: Movement buttons, speed control
- **Side Effects**: Sends movement commands
- **Dependencies**: WebSocket client
- **Patterns**: Control panel pattern, command pattern

#### `src/components/ControlButtons.tsx`
- **Purpose**: Control button component
- **Key Functions**: Reusable control buttons
- **Side Effects**: Triggers commands
- **Dependencies**: Command context
- **Patterns**: Button component pattern

#### `src/components/CameraControlPanel.tsx`
- **Purpose**: Camera control panel
- **Key Functions**: Camera settings, servo control
- **Side Effects**: Controls camera/servos
- **Dependencies**: WebSocket client
- **Patterns**: Control panel pattern

#### `src/components/SensorDashboard.tsx`
- **Purpose**: Sensor data dashboard
- **Key Functions**: Displays sensor readings
- **Side Effects**: Connects to sensor WebSockets
- **Dependencies**: Sensor WebSocket clients
- **Patterns**: Dashboard pattern, real-time data pattern

#### `src/components/UltrasonicSensorStatus.tsx`
- **Purpose**: Ultrasonic sensor display
- **Key Functions**: Shows distance readings
- **Side Effects**: Connects to ultrasonic WebSocket
- **Dependencies**: Ultrasonic WebSocket
- **Patterns**: Sensor component pattern

#### `src/components/LineTrackerStatus.tsx`
- **Purpose**: Line tracker status display
- **Key Functions**: Shows line tracking state
- **Side Effects**: Connects to line tracker WebSocket
- **Dependencies**: Line tracker WebSocket
- **Patterns**: Sensor component pattern

#### `src/components/LedControl.tsx`
- **Purpose**: LED control component
- **Key Functions**: LED color selection, pattern selection
- **Side Effects**: Controls LED hardware
- **Dependencies**: Lighting API
- **Patterns**: Control component pattern

#### `src/components/LedModal.tsx`
- **Purpose**: LED control modal
- **Key Functions**: Modal dialog for LED control
- **Side Effects**: Opens/closes modal
- **Dependencies**: Modal library, LED control
- **Patterns**: Modal pattern

#### `src/components/ColorWheel.tsx`
- **Purpose**: Color picker component
- **Key Functions**: Color selection UI
- **Side Effects**: None (UI only)
- **Dependencies**: react-color
- **Patterns**: Color picker pattern

#### `src/components/PerformanceDashboard.tsx`
- **Purpose**: Performance metrics dashboard
- **Key Functions**: Displays performance metrics
- **Side Effects**: Fetches performance data
- **Dependencies**: Performance API
- **Patterns**: Dashboard pattern, metrics visualization

#### `src/components/SpeedControl.tsx`
- **Purpose**: Speed control component
- **Key Functions**: Speed adjustment slider/buttons
- **Side Effects**: Sends speed commands
- **Dependencies**: WebSocket client
- **Patterns**: Control component pattern

#### `src/components/CommandLog.tsx`
- **Purpose**: Command log display
- **Key Functions**: Shows command history
- **Side Effects**: Displays logs
- **Dependencies**: Command context
- **Patterns**: Log component pattern

#### `src/components/ErrorBoundary.tsx`
- **Purpose**: React error boundary
- **Key Functions**: Catches React errors, displays fallback
- **Side Effects**: Prevents app crashes
- **Dependencies**: React error boundary API
- **Patterns**: Error boundary pattern

#### `src/components/NetworkWizard.tsx`
- **Purpose**: Network configuration wizard
- **Key Functions**: Guides network setup
- **Side Effects**: Changes network configuration
- **Dependencies**: Network APIs
- **Patterns**: Wizard pattern, configuration pattern

#### `src/components/NetworkProfileSelector.tsx`
- **Purpose**: Network profile selector
- **Key Functions**: Selects network profile (local/lan/tailscale)
- **Side Effects**: Changes active profile
- **Dependencies**: Configuration system
- **Patterns**: Selector pattern, configuration pattern

#### `src/components/ServiceStatusBar.tsx`
- **Purpose**: Service status indicator bar
- **Key Functions**: Shows service connection status
- **Side Effects**: Monitors service health
- **Dependencies**: Service health APIs
- **Patterns**: Status indicator pattern

#### ROS Components (`src/components/ros/`)

#### `src/components/ros/ROSManagementPanel.tsx`
- **Purpose**: ROS2 management panel
- **Key Functions**: ROS2 node management, topic monitoring
- **Side Effects**: Interacts with ROS2 bridge
- **Dependencies**: ROS2 WebSocket bridge
- **Patterns**: Management panel pattern

#### `src/components/ros/TelemetryVisualization.tsx`
- **Purpose**: ROS2 telemetry visualization
- **Key Functions**: Displays ROS2 telemetry data
- **Side Effects**: Subscribes to ROS2 topics
- **Dependencies**: ROS2 bridge
- **Patterns**: Visualization pattern

#### `src/components/ros/ROS2TopicViewer.tsx`
- **Purpose**: ROS2 topic viewer
- **Key Functions**: Lists and monitors ROS2 topics
- **Side Effects**: Queries ROS2 topics
- **Dependencies**: ROS2 bridge
- **Patterns**: Topic viewer pattern

### Hooks (`src/hooks/`)

#### `src/hooks/useWebSocket.ts`
- **Purpose**: WebSocket hook
- **Key Functions**: WebSocket connection management
- **Side Effects**: Manages WebSocket lifecycle
- **Dependencies**: WebSocket API
- **Patterns**: Custom hook pattern, WebSocket pattern

#### `src/hooks/useCapabilities.ts`
- **Purpose**: Capability detection hook
- **Key Functions**: Fetches system capabilities
- **Side Effects**: Queries capability API
- **Dependencies**: Capability API
- **Patterns**: Custom hook pattern, capability pattern

#### `src/hooks/useROS2Status.ts`
- **Purpose**: ROS2 status hook
- **Key Functions**: Monitors ROS2 connection status
- **Side Effects**: Queries ROS2 status
- **Dependencies**: ROS2 bridge
- **Patterns**: Custom hook pattern, status monitoring

### Context (`src/context/`)

#### `src/context/CommandContext.tsx`
- **Purpose**: Command context provider
- **Key Functions**: Manages command state, command log
- **Side Effects**: Maintains command history
- **Dependencies**: React Context API
- **Patterns**: Context pattern, state management pattern

#### `src/context/CapabilityContext.tsx`
- **Purpose**: Capability context provider
- **Key Functions**: Provides capability data to components
- **Side Effects**: Fetches capabilities
- **Dependencies**: Capability API
- **Patterns**: Context pattern

#### `src/context/MacroContext.tsx`
- **Purpose**: Macro context provider
- **Key Functions**: Manages macro commands
- **Side Effects**: Stores macro definitions
- **Dependencies**: React Context API
- **Patterns**: Context pattern

### Configuration (`src/config/`)

#### `src/config/gateway.ts`
- **Purpose**: Gateway URL configuration
- **Key Functions**: Resolves URLs based on network profile
- **Side Effects**: None (configuration only)
- **Dependencies**: Environment variables
- **Patterns**: Configuration pattern, profile-based routing

#### `src/config/environment.ts`
- **Purpose**: Environment configuration
- **Key Functions**: Environment variable management
- **Side Effects**: None
- **Dependencies**: Environment variables
- **Patterns**: Configuration pattern

### Utils (`src/utils/`)

#### `src/utils/autonomyApi.ts`
- **Purpose**: Autonomy API client
- **Key Functions**: Autonomy command interface
- **Side Effects**: Sends autonomy commands
- **Dependencies**: WebSocket client
- **Patterns**: API client pattern, wire pattern (mock/real)

### Redux (`src/redux/`)

#### `src/redux/store.ts`
- **Purpose**: Redux store configuration
- **Key Functions**: Creates Redux store
- **Side Effects**: Manages global state
- **Dependencies**: Redux Toolkit
- **Patterns**: Redux pattern, state management pattern

#### `src/redux/reducers/`
- **Purpose**: Redux reducers
- **Key Functions**: State update logic
- **Side Effects**: Updates Redux state
- **Dependencies**: Redux Toolkit
- **Patterns**: Reducer pattern

### Tests (`tests/`)

#### `tests/*.test.tsx`
- **Purpose**: Component unit tests
- **Key Functions**: Tests React components
- **Side Effects**: None (test only)
- **Dependencies**: Jest, React Testing Library
- **Patterns**: Unit testing pattern

#### `cypress/e2e/`
- **Purpose**: End-to-end tests
- **Key Functions**: Full user flow tests
- **Side Effects**: May start test servers
- **Dependencies**: Cypress
- **Patterns**: E2E testing pattern

---

## ROS2 Files

### Launch Files (`ros/launch/`)

#### `ros/launch/omega_full.launch.py`
- **Purpose**: Full Omega robot launch file
- **Key Functions**: Launches all robot nodes
- **Side Effects**: Starts ROS2 nodes
- **Dependencies**: ROS2 Humble/Rolling
- **Patterns**: ROS2 launch pattern

#### `ros/launch/omega_brain.launch.py`
- **Purpose**: Brain/control nodes launch
- **Key Functions**: Launches control nodes
- **Side Effects**: Starts control nodes
- **Dependencies**: ROS2
- **Patterns**: ROS2 launch pattern

#### `ros/launch/omega_camera.launch.py`
- **Purpose**: Camera nodes launch
- **Key Functions**: Launches camera publisher
- **Side Effects**: Starts camera nodes
- **Dependencies**: ROS2
- **Patterns**: ROS2 launch pattern

#### `ros/launch/multidevice_setup.launch.py`
- **Purpose**: Multi-device ROS2 setup
- **Key Functions**: Configures multi-device communication
- **Side Effects**: Sets up DDS configuration
- **Dependencies**: ROS2, CycloneDDS
- **Patterns**: Multi-device pattern, DDS configuration

### ROS2 Package (`ros/src/omega_robot/`)

#### `ros/src/omega_robot/omega_robot/telemetry_publisher.py`
- **Purpose**: ROS2 telemetry publisher node
- **Key Functions**: Publishes heartbeat telemetry on `/omega/telemetry`
- **Side Effects**: Publishes ROS2 messages
- **Dependencies**: rclpy, std_msgs
- **Patterns**: ROS2 publisher pattern

#### `ros/src/omega_robot/omega_robot/telemetry_listener.py`
- **Purpose**: ROS2 telemetry listener node
- **Key Functions**: Subscribes to `/omega/telemetry`
- **Side Effects**: Logs telemetry messages
- **Dependencies**: rclpy, std_msgs
- **Patterns**: ROS2 subscriber pattern

#### `ros/src/omega_robot/omega_robot/sensor_data_publisher.py`
- **Purpose**: Sensor data publisher
- **Key Functions**: Publishes sensor readings to ROS2 topics
- **Side Effects**: Publishes sensor data
- **Dependencies**: rclpy, sensor_msgs
- **Patterns**: ROS2 publisher pattern, sensor pattern

#### `ros/src/omega_robot/omega_robot/robot_controller.py`
- **Purpose**: ROS2 robot controller node
- **Key Functions**: Subscribes to `/cmd_vel`, controls motors
- **Side Effects**: Controls robot hardware
- **Dependencies**: rclpy, geometry_msgs
- **Patterns**: ROS2 controller pattern, cmd_vel pattern

#### `ros/src/omega_robot/omega_robot/camera_publisher.py`
- **Purpose**: Camera image publisher
- **Key Functions**: Publishes camera frames to ROS2
- **Side Effects**: Publishes image messages
- **Dependencies**: rclpy, sensor_msgs
- **Patterns**: ROS2 publisher pattern, image pattern

#### `ros/src/omega_robot/omega_robot/vision_processor.py`
- **Purpose**: Vision processing node
- **Key Functions**: Processes camera images, publishes detections
- **Side Effects**: Processes images
- **Dependencies**: rclpy, opencv-python
- **Patterns**: ROS2 node pattern, computer vision pattern

#### `ros/src/omega_robot/omega_robot/path_planner.py`
- **Purpose**: Path planning node
- **Key Functions**: Plans paths, publishes waypoints
- **Side Effects**: Computes paths
- **Dependencies**: rclpy, nav_msgs
- **Patterns**: ROS2 node pattern, path planning pattern

#### `ros/src/omega_robot/omega_robot/obstacle_avoidance_action.py`
- **Purpose**: Obstacle avoidance action server
- **Key Functions**: ROS2 action for obstacle avoidance
- **Side Effects**: Controls robot to avoid obstacles
- **Dependencies**: rclpy, actionlib
- **Patterns**: ROS2 action pattern

#### `ros/src/omega_robot/omega_robot/follow_line_action.py`
- **Purpose**: Line following action server
- **Key Functions**: ROS2 action for line following
- **Side Effects**: Controls robot to follow line
- **Dependencies**: rclpy, actionlib
- **Patterns**: ROS2 action pattern

#### `ros/src/omega_robot/omega_robot/navigate_to_goal_action.py`
- **Purpose**: Navigation action server
- **Key Functions**: ROS2 action for navigation
- **Side Effects**: Controls robot navigation
- **Dependencies**: rclpy, actionlib, nav_msgs
- **Patterns**: ROS2 action pattern, navigation pattern

#### `ros/src/omega_robot/omega_robot/enhanced_telemetry.py`
- **Purpose**: Enhanced telemetry publisher
- **Key Functions**: Publishes rich telemetry data
- **Side Effects**: Publishes telemetry
- **Dependencies**: rclpy, custom messages
- **Patterns**: ROS2 publisher pattern

#### `ros/src/omega_robot/omega_robot/system_capabilities.py`
- **Purpose**: System capability detection
- **Key Functions**: Detects and publishes system capabilities
- **Side Effects**: Queries system
- **Dependencies**: rclpy
- **Patterns**: ROS2 node pattern, capability pattern

#### `ros/src/omega_robot/package.xml`
- **Purpose**: ROS2 package manifest
- **Key Functions**: Defines package dependencies
- **Side Effects**: None (metadata)
- **Dependencies**: ROS2 build system
- **Patterns**: ROS2 package pattern

#### `ros/src/omega_robot/setup.py`
- **Purpose**: Python package setup
- **Key Functions**: Installs Python nodes
- **Side Effects**: Installs package
- **Dependencies**: setuptools, ROS2
- **Patterns**: Python package pattern

### ROS2 Scripts (`ros/scripts/`)

#### `ros/scripts/a_star.py`, `ros/scripts/a_star_ros.py`
- **Purpose**: A* path planning algorithm
- **Key Functions**: A* pathfinding implementation
- **Side Effects**: Computes paths
- **Dependencies**: numpy
- **Patterns**: Path planning pattern, A* algorithm

#### `ros/scripts/d_star_lite.py`, `ros/scripts/d_star_lite_ros.py`
- **Purpose**: D* Lite path planning
- **Key Functions**: Dynamic path replanning
- **Side Effects**: Computes paths
- **Dependencies**: numpy
- **Patterns**: Path planning pattern, D* Lite algorithm

#### `ros/scripts/rrt.py`, `ros/scripts/rrt_ros.py`
- **Purpose**: RRT path planning
- **Key Functions**: Rapidly-exploring Random Tree
- **Side Effects**: Computes paths
- **Dependencies**: numpy
- **Patterns**: Path planning pattern, RRT algorithm

#### `ros/scripts/autonomous_driving.py`
- **Purpose**: Autonomous driving script
- **Key Functions**: Autonomous driving logic
- **Side Effects**: Controls robot
- **Dependencies**: ROS2, sensors
- **Patterns**: Autonomous behavior pattern

#### `ros/scripts/sensor_fusion.py`
- **Purpose**: Sensor fusion script
- **Key Functions**: Combines sensor data
- **Side Effects**: Processes sensor data
- **Dependencies**: numpy, sensors
- **Patterns**: Sensor fusion pattern

---

## Scripts

### Setup Scripts (`scripts/`)

#### `scripts/check_endpoints.sh`
- **Purpose**: Profile-aware endpoint checker
- **Key Functions**: Tests connectivity to robot services
- **Side Effects**: Pings hosts, tests TCP ports
- **Dependencies**: bash, curl, nc/ping
- **Patterns**: Health check pattern, network testing pattern

#### `scripts/start_robot.sh`
- **Purpose**: Start robot services
- **Key Functions**: Starts ROS, backend services, UI
- **Side Effects**: Starts multiple services
- **Dependencies**: SSH, ROS, backend services
- **Patterns**: Orchestration pattern

#### `scripts/setup_ros2_laptop.sh`
- **Purpose**: Setup ROS2 on laptop
- **Key Functions**: Installs ROS2 Humble/Rolling
- **Side Effects**: Installs ROS2 system
- **Dependencies**: Ubuntu, apt
- **Patterns**: Installation script pattern

#### `scripts/setup_multidevice_ros2.sh`
- **Purpose**: Multi-device ROS2 setup
- **Key Functions**: Configures CycloneDDS for multi-device
- **Side Effects**: Configures DDS
- **Dependencies**: ROS2, CycloneDDS
- **Patterns**: Multi-device configuration pattern

#### `scripts/connect_robot_bluetooth.sh`
- **Purpose**: Bluetooth PAN connection
- **Key Functions**: Connects Mac to iPhone PAN, triggers Pi helper
- **Side Effects**: Establishes Bluetooth connection
- **Dependencies**: blueutil, SSH
- **Patterns**: Network connection pattern

#### `scripts/pi_connect_phone_pan.sh`
- **Purpose**: Pi Bluetooth PAN connection
- **Key Functions**: Connects Pi to iPhone PAN
- **Side Effects**: Establishes Bluetooth PAN
- **Dependencies**: bluez, rfkill
- **Patterns**: Network connection pattern

#### `scripts/verify_integration.sh`
- **Purpose**: Integration verification
- **Key Functions**: Tests system integration
- **Side Effects**: Runs integration tests
- **Dependencies**: Test frameworks
- **Patterns**: Verification pattern

---

## Docker Configuration

### ROS2 Docker (`docker/ros2_robot/`)

#### `docker/ros2_robot/Dockerfile`
- **Purpose**: ROS2 Humble Docker image
- **Key Functions**: Builds ROS2 container for Pi
- **Side Effects**: Creates Docker image
- **Dependencies**: Docker, ROS2 Humble base image
- **Patterns**: Containerization pattern

#### `docker/ros2_robot/docker-compose.yml`
- **Purpose**: ROS2 Docker Compose configuration
- **Key Functions**: Orchestrates ROS2 containers
- **Side Effects**: Starts ROS2 containers
- **Dependencies**: docker-compose
- **Patterns**: Container orchestration pattern

#### `docker/ros2_robot/config/cyclonedds.xml`
- **Purpose**: CycloneDDS configuration
- **Key Functions**: Configures DDS for multi-device communication
- **Side Effects**: Configures DDS middleware
- **Dependencies**: CycloneDDS
- **Patterns**: DDS configuration pattern

---

## Documentation

### Architecture Documentation

#### `README.md`
- **Purpose**: Main repository documentation
- **Key Functions**: Overview, setup instructions, architecture
- **Side Effects**: None (documentation)
- **Dependencies**: None
- **Patterns**: Documentation pattern

#### `ROS2_ARCHITECTURE.md`
- **Purpose**: ROS2 architecture documentation
- **Key Functions**: Explains ROS2 integration
- **Side Effects**: None
- **Dependencies**: None
- **Patterns**: Architecture documentation pattern

#### `ROS2_MULTIDEVICE_SETUP.md`
- **Purpose**: Multi-device setup guide
- **Key Functions**: Instructions for multi-device ROS2
- **Side Effects**: None
- **Dependencies**: None
- **Patterns**: Setup guide pattern

### Phase Documentation

#### `PHASE2_CAMERA_INTEGRATION.md`
- **Purpose**: Camera integration phase documentation
- **Key Functions**: Documents camera integration
- **Side Effects**: None
- **Dependencies**: None
- **Patterns**: Phase documentation pattern

#### `PHASE3_AUTONOMOUS_BEHAVIORS.md`
- **Purpose**: Autonomous behaviors phase documentation
- **Key Functions**: Documents autonomy implementation
- **Side Effects**: None
- **Dependencies**: None
- **Patterns**: Phase documentation pattern

#### `PHASE4_NAVIGATION_SLAM.md`
- **Purpose**: Navigation and SLAM phase documentation
- **Key Functions**: Documents navigation/SLAM implementation
- **Side Effects**: None
- **Dependencies**: None
- **Patterns**: Phase documentation pattern

---

## Architectural Patterns Summary

### Backend Patterns

1. **WebSocket Server Pattern**: Used extensively for real-time communication
2. **API Gateway Pattern**: Gateway API aggregates multiple services
3. **Hardware Abstraction Pattern**: GPIO, motor, sensor abstractions
4. **Strategy Pattern**: Camera backend selection (Picamera2 vs V4L2)
5. **Command Pattern**: Command processing for robot control
6. **State Machine Pattern**: Autonomy controller state management
7. **Observer Pattern**: WebSocket message broadcasting
8. **Factory Pattern**: Hardware factory for mock/real implementations
9. **Bridge Pattern**: ROS2-WebSocket bridge
10. **Cache Pattern**: Redis caching with memory fallback

### Frontend Patterns

1. **Component Composition Pattern**: React component hierarchy
2. **Context Pattern**: React Context for state management
3. **Custom Hook Pattern**: Reusable React hooks
4. **Provider Pattern**: Context providers
5. **Error Boundary Pattern**: React error boundaries
6. **Profile-based Configuration**: Network profile routing
7. **WebSocket Pattern**: Real-time WebSocket communication
8. **Mock Pattern**: Mock WebSocket for development

### ROS2 Patterns

1. **Publisher/Subscriber Pattern**: ROS2 topic communication
2. **Action Pattern**: ROS2 action servers/clients
3. **Node Pattern**: ROS2 node architecture
4. **Launch Pattern**: ROS2 launch file configuration
5. **Multi-device Pattern**: Distributed ROS2 setup

### Infrastructure Patterns

1. **Containerization Pattern**: Docker containers
2. **Reverse Proxy Pattern**: Nginx reverse proxy
3. **Service-oriented Architecture**: Microservices
4. **Health Check Pattern**: Service health monitoring
5. **Configuration Pattern**: Environment-based configuration

---

## External Dependencies Summary

### Backend Dependencies

- **Python**: 3.11+ (FastAPI, Flask, asyncio, websockets)
- **Go**: 1.22+ (gorilla/websocket, GPIO libraries)
- **Hardware**: RPi.GPIO, picamera2, opencv-python
- **ROS2**: rclpy, ROS2 Humble/Rolling
- **Caching**: Redis (optional)
- **Monitoring**: psutil

### Frontend Dependencies

- **Next.js**: 14+
- **React**: 18+
- **TypeScript**: 5+
- **Redux Toolkit**: State management
- **Tailwind CSS**: Styling
- **WebSocket**: ws library

### Infrastructure Dependencies

- **Docker**: Containerization
- **Nginx**: Reverse proxy
- **CycloneDDS**: ROS2 DDS middleware

---

This summary provides a comprehensive overview of the Omega-Code repository structure, patterns, and dependencies. Each file serves a specific purpose in the robotics control stack, from low-level hardware control to high-level autonomous behaviors.

