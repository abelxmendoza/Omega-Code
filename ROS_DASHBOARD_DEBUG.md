# ROS 2 Dashboard with Built-in Debug Tools

## Overview

The ROS 2 Dashboard (`/ros`) provides a comprehensive interface for managing ROS 2 Docker containers with built-in debugging tools that leverage browser DevTools.

## Features

### 🎯 Core Features
- **ROS Management Panel**: Start/stop/restart containers, view logs, monitor status
- **Telemetry Visualization**: Real-time display of `/omega/telemetry` topic
- **Auto-refresh**: Configurable polling interval (1s, 2s, 5s, 10s)
- **WebSocket Monitoring**: Visual connection status with reconnect attempts

### 🔧 Built-in Debug Tools

#### 1. **Console Logging**
All actions are logged to the browser console with color-coded prefixes:
- 🟢 **Green** `[ROS Debug]` - Info messages
- 🔴 **Red** `[ROS Error]` - Errors
- 🟡 **Yellow** `[ROS Warning]` - Warnings
- 🔵 **Blue** `[ROS Network]` - API requests
- 🟣 **Purple** `[ROS WebSocket]` - WebSocket events

#### 2. **Network Monitoring**
Open DevTools → Network tab to see:
- All API requests (`/api/ros/*`)
- Request/response headers and bodies
- Timing information (latency, duration)
- Filter by "WS" to see WebSocket frames

#### 3. **WebSocket Debugging**
- Real-time WebSocket message inspection
- Connection state monitoring
- Reconnection attempt tracking
- Message payload inspection

#### 4. **In-Page Debug Panel**
- Live debug log viewer (last 200 entries)
- Color-coded by type (API, WebSocket, Error, Status)
- Expandable data inspection
- Timestamp for each entry

## Usage

### Accessing the Dashboard

1. **From Header**: Click the "ROS" badge in the header
2. **Direct URL**: Navigate to `/ros`
3. **Quick Access**: Bookmark for easy access

### Using DevTools

1. **Open DevTools**: Press `F12` or `Cmd+Option+I` (Mac) / `Ctrl+Shift+I` (Windows/Linux)
2. **Console Tab**: View all debug logs
   - Filter by `[ROS` to see only ROS-related logs
   - Use console filtering: `[ROS Debug]`, `[ROS Error]`, etc.
3. **Network Tab**: Monitor API calls
   - Filter by `/api/ros` to see ROS API calls
   - Click any request to see headers, payload, response
4. **WS Filter**: See WebSocket frames
   - Filter by "WS" to see WebSocket connections
   - Click to inspect frames sent/received
5. **Performance Tab**: Analyze timing
   - Record while interacting with dashboard
   - See refresh intervals, API call timing

### Debug Mode

Enable debug mode for verbose logging:

**Option 1: Environment Variable**
```bash
NEXT_PUBLIC_ROS_DEBUG=1 npm run dev
```

**Option 2: Browser Console**
```javascript
localStorage.setItem('ros_debug', 'true');
// Refresh page
```

**Disable:**
```javascript
localStorage.removeItem('ros_debug');
```

## Debug Log Types

### API Calls
```javascript
[ROS Network] GET /api/ros/status { containers: 2, topics: 1 }
[ROS Network] POST /api/ros/control { action: "start", service: null }
```

### WebSocket Events
```javascript
[ROS WebSocket] connect ws://localhost:7070/ws/ros/telemetry
[ROS WebSocket] open Connected
[ROS WebSocket] message { topic: "/omega/telemetry", data: "..." }
[ROS WebSocket] close { code: 1000, reason: "Normal closure" }
```

### Status Updates
```javascript
[ROS Debug] Status fetched in 45.23ms { containers: 2, topics: 1 }
[ROS Debug] Found 3 topics ["/omega/telemetry", ...]
```

### Errors
```javascript
[ROS Error] Failed to fetch status { error: "Connection refused" }
[ROS Error] WebSocket error [Error object]
```

## Troubleshooting with DevTools

### Container Won't Start

1. **Check Network Tab**:
   - Look for `/api/ros/control` POST request
   - Check response status (should be 200)
   - Inspect response body for error details

2. **Check Console**:
   - Look for `[ROS Error]` entries
   - Error messages will show what went wrong

### Telemetry Not Showing

1. **Check WebSocket Tab**:
   - Filter Network tab by "WS"
   - Verify connection to `/ws/ros/telemetry`
   - Check if frames are being received

2. **Check Console**:
   - Look for `[ROS WebSocket]` entries
   - Check connection status messages
   - Look for reconnection attempts

### Performance Issues

1. **Check Performance Tab**:
   - Record while using dashboard
   - Look for long-running operations
   - Check refresh interval timing

2. **Check Network Tab**:
   - Look for slow API calls
   - Check timing information
   - Identify bottlenecks

## Best Practices

1. **Keep DevTools Open**: While developing/debugging ROS integration
2. **Use Console Filters**: Filter by `[ROS` to see only ROS logs
3. **Monitor Network Tab**: Watch for failed requests or slow responses
4. **Check WebSocket Tab**: Monitor connection stability
5. **Use Debug Panel**: Quick view of recent activity without DevTools

## Advanced Debugging

### Intercept API Calls

Use DevTools → Network → Right-click request → Copy → Copy as cURL:
```bash
curl -X POST http://localhost:7070/api/ros/control \
  -H "Content-Type: application/json" \
  -d '{"action":"start"}'
```

### Inspect WebSocket Messages

1. Open Network tab → Filter "WS"
2. Click WebSocket connection
3. View "Messages" tab
4. See all frames sent/received with timestamps

### Performance Profiling

1. Open Performance tab
2. Click Record
3. Interact with dashboard
4. Stop recording
5. Analyze timeline for bottlenecks

## Example Debug Session

```
1. Open /ros page
2. Open DevTools (F12)
3. Go to Console tab
4. Filter by "[ROS"
5. Click "Start All" button
6. Watch console for:
   - [ROS Network] POST /api/ros/control
   - [ROS Debug] Successfully started all containers
   - [ROS Debug] Status fetched in 45ms
7. Check Network tab for actual HTTP request
8. Verify WebSocket connection in Network → WS filter
```

## Quick Reference

| Action | DevTools Location | What to Look For |
|--------|------------------|------------------|
| Debug logs | Console tab | `[ROS Debug]`, `[ROS Error]` |
| API calls | Network tab → Filter `/api/ros` | Request/response, status codes |
| WebSocket | Network tab → Filter "WS" | Connection status, frames |
| Performance | Performance tab | Timing, bottlenecks |
| Errors | Console tab → Filter "Error" | Error messages, stack traces |

This dashboard is designed to work seamlessly with browser DevTools for comprehensive debugging and monitoring of ROS 2 containers and telemetry.

