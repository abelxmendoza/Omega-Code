# Environment Variables

This document describes the environment variables needed to run the robot controller application.

## Frontend (Next.js UI)

### Required Variables
Create a `.env.local` file in the `ui/robot-controller-ui/` directory:

```bash
# WebSocket URLs for backend services
NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT=ws://localhost:8001/ws
NEXT_PUBLIC_BACKEND_WS_URL_VIDEO=ws://localhost:8002/ws
NEXT_PUBLIC_BACKEND_WS_URL_SENSORS=ws://localhost:8003/ws

# API endpoints
NEXT_PUBLIC_BACKEND_API_URL=http://localhost:8000
NEXT_PUBLIC_VIDEO_API_URL=http://localhost:8002
```

### Optional Variables
```bash
# Development settings
NEXT_PUBLIC_MOCK_WS=true  # Use mock WebSocket connections for development
NODE_ENV=development
```

## Backend (Python Services)

### Required Variables
Create a `.env` file in the `servers/robot-controller-backend/` directory:

```bash
# Server configuration
HOST=0.0.0.0
PORT=8000
DEBUG=true

# Hardware configuration (Raspberry Pi specific)
GPIO_PIN_LED=18
GPIO_PIN_MOTOR_LEFT=20
GPIO_PIN_MOTOR_RIGHT=21
```

### Optional Variables
```bash
# Logging
LOG_LEVEL=INFO
LOG_FILE=robot.log

# Camera settings
CAMERA_WIDTH=640
CAMERA_HEIGHT=480
CAMERA_FPS=30
```

## Quick Setup

1. **Frontend**: Copy the example variables above to `ui/robot-controller-ui/.env.local`
2. **Backend**: Copy the example variables above to `servers/robot-controller-backend/.env`
3. **Development**: Set `NEXT_PUBLIC_MOCK_WS=true` to use mock connections

## Troubleshooting

- **"WS URL missing"**: Make sure all `NEXT_PUBLIC_BACKEND_WS_URL_*` variables are set
- **Connection refused**: Ensure backend services are running on the specified ports
- **Hardware errors**: Check GPIO pin assignments match your robot's wiring
