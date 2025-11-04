# ROS 2 Docker Integration - UI Guide

## Overview

The UI now includes comprehensive ROS 2 Docker management and telemetry visualization capabilities. This integration makes it easy to start, stop, and monitor ROS 2 containers from the web interface.

## Features

### 1. **ROS 2 Management Panel** (`components/ros/ROSManagementPanel.tsx`)
- **Container Status**: Real-time view of all ROS 2 containers (telemetry_publisher, telemetry_listener)
- **Quick Actions**: Start, stop, and restart containers with one click
- **Logs Viewer**: View container logs directly in the UI
- **Topic List**: See all available ROS 2 topics when containers are running
- **Auto-refresh**: Status updates every 5 seconds

### 2. **Telemetry Visualization** (`components/ros/TelemetryVisualization.tsx`)
- **Live Telemetry**: Real-time display of `/omega/telemetry` topic messages
- **WebSocket Connection**: Automatic reconnection with exponential backoff
- **Message History**: Keep last 100 messages for analysis
- **Connection Status**: Visual indicator of WebSocket connection state

### 3. **Backend API** (`api/ros_routes.py`)
- **GET `/api/ros/status`**: Get container status and topics
- **POST `/api/ros/control`**: Start/stop/restart containers
- **GET `/api/ros/logs/{service}`**: Fetch container logs
- **GET `/api/ros/topics`**: List ROS 2 topics
- **WebSocket `/api/ros/telemetry`**: Bridge ROS 2 telemetry to UI

### 4. **Gateway Proxy** (`servers/gateway_api.py`)
- All ROS endpoints proxied through gateway for unified access
- WebSocket proxy for telemetry stream
- Consistent with existing gateway architecture

## Usage

### Adding to Your Pages

```tsx
import { ROSManagementPanel, TelemetryVisualization } from '@/components/ros';

// In your component:
<ROSManagementPanel className="mb-4" />
<TelemetryVisualization className="mb-4" />
```

### Programmatic API Usage

```typescript
import { getROSStatus, controlROSContainer, getROSLogs } from '@/utils/rosApi';

// Get status
const status = await getROSStatus();
console.log('Containers:', status.containers);
console.log('Topics:', status.topics);

// Start containers
await controlROSContainer({ action: 'start' });

// Stop specific container
await controlROSContainer({ action: 'stop', service: 'telemetry_publisher' });

// Get logs
const logs = await getROSLogs('telemetry_publisher', 100);
```

### WebSocket Telemetry

```typescript
import { createROSTelemetryWebSocket } from '@/utils/rosApi';

const ws = await createROSTelemetryWebSocket(
  (data) => console.log('Telemetry:', data),
  (error) => console.error('Error:', error),
  () => console.log('Disconnected')
);
```

## Integration Points

### ServiceStatusBar Integration
Add ROS status to the existing service status bar:

```tsx
// In ServiceStatusBar.tsx
import { useHttpStatus } from '../hooks/useHttpStatus';
import { buildGatewayUrl } from '@/config/gateway';

const rosStatus = useHttpStatus(
  buildGatewayUrl('/api/ros/status'),
  { intervalMs: 5000 }
);
```

### Header Quick Actions
Add ROS quick actions to Header component:

```tsx
// In Header.tsx
import { controlROSContainer } from '@/utils/rosApi';

<button onClick={() => controlROSContainer({ action: 'start' })}>
  Start ROS
</button>
```

## Reliability Features

1. **Automatic Reconnection**: WebSocket connections auto-reconnect with exponential backoff
2. **Error Handling**: Graceful error messages and fallback states
3. **Status Polling**: Container status refreshes every 5 seconds
4. **Timeout Protection**: All API calls have reasonable timeouts
5. **Network Profile Awareness**: Uses centralized gateway URL resolution

## Startup Workflow

### Recommended Startup Sequence:

1. **Start Backend Services** (if not already running):
   ```bash
   cd servers/robot-controller-backend
   ./scripts/run_gateway.sh
   ```

2. **Start ROS Containers via UI**:
   - Open ROS Management Panel
   - Click "Start All" button
   - Verify containers are running

3. **Monitor Telemetry**:
   - Open Telemetry Visualization component
   - Verify connection status is green
   - Watch for incoming messages

### One-Click Startup (Future Enhancement):
Create a startup script that:
- Checks Docker availability
- Starts gateway if needed
- Starts ROS containers
- Verifies all services are healthy

## Environment Variables

No additional environment variables needed - uses existing gateway configuration:
- `NEXT_PUBLIC_GATEWAY_HOST_*` (for gateway URL resolution)
- `MAIN_API_URL` (backend API URL, defaults to `http://127.0.0.1:8000`)
- `MAIN_API_WS_URL` (WebSocket URL, defaults to `ws://127.0.0.1:8000`)

## Troubleshooting

### Containers Not Starting
- Check Docker is running: `docker ps`
- Verify docker-compose.yml path is correct
- Check logs in ROS Management Panel

### Telemetry Not Showing
- Verify containers are running
- Check WebSocket connection status
- Ensure `/omega/telemetry` topic is being published

### Gateway Errors
- Verify gateway is running on port 7070
- Check backend API is accessible
- Review gateway logs for proxy errors

## Next Steps

1. **Add ROS status to ServiceStatusBar** - Visual indicator in status bar
2. **Create ROS Dashboard Page** - Dedicated page for ROS management
3. **Add ROS Node Controls** - Start/stop individual nodes
4. **Topic Explorer** - Browse and subscribe to any ROS topic
5. **Command Publisher** - Publish commands to ROS topics from UI

