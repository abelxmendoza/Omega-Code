# OmegaOS Service Orchestrator

Unified service management system for Omega-1 robot. Provides automatic process launching, health monitoring, auto-recovery, and REST API control.

## 🎯 Features

- ✅ **Declarative Service Registry** - Define services in JSON, no code changes needed
- ✅ **Auto-Start Services** - Services start automatically on boot
- ✅ **Auto-Recovery** - Crashed services restart based on policy
- ✅ **Health Checks** - Monitor service health and report issues
- ✅ **Process Monitoring** - Track CPU, RAM, PID for each service
- ✅ **REST API** - Control services via `/api/services/*` endpoints
- ✅ **Web UI** - Manage services from `/services` page
- ✅ **Log Management** - View service logs via API and UI
- ✅ **Crash Detection** - Detect and handle service crashes
- ✅ **Crash Loop Protection** - Cooldown prevents restart storms

## 📁 Structure

```
omega_services/
├── service_manager.py      # Top-level orchestrator
├── process_supervisor.py    # Process monitoring & restart
├── health_checks.py         # Health check functions
├── service_registry.json    # Service definitions
├── logs/                   # Service logs
│   ├── {service}.stdout.log
│   └── {service}.stderr.log
└── __init__.py
```

## 🚀 Quick Start

### 1. Install Dependencies

```bash
pip install psutil
```

### 2. Configure Services

Edit `service_registry.json` to define your services:

```json
{
  "services": [
    {
      "name": "movement_ws_server",
      "display_name": "Movement Server",
      "type": "python",
      "cmd": "python3",
      "args": ["movement/movement_ws_server.py"],
      "autostart": true,
      "restart_policy": "always"
    }
  ]
}
```

### 3. Start Orchestrator

**Manual Start:**
```bash
cd servers/robot_controller_backend
python3 -m omega_services.service_manager
```

**Systemd Service (Recommended):**
```bash
# Copy service file
sudo cp omega_services/omega-orchestrator.service /etc/systemd/system/

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable omega-orchestrator
sudo systemctl start omega-orchestrator

# Check status
sudo systemctl status omega-orchestrator
```

## 📡 API Endpoints

### List All Services
```bash
GET /api/services/list
```

Response:
```json
{
  "ok": true,
  "services": [
    {
      "name": "movement_ws_server",
      "status": "running",
      "pid": 1234,
      "health": {"healthy": true, "message": "OK"}
    }
  ]
}
```

### Get Service Status
```bash
GET /api/services/status/{name}
```

### Start Service
```bash
POST /api/services/start/{name}
```

### Stop Service
```bash
POST /api/services/stop/{name}?force=false
```

### Restart Service
```bash
POST /api/services/restart/{name}
```

### Get Service Logs
```bash
GET /api/services/logs/{name}?lines=50
```

### Check Service Health
```bash
GET /api/services/health/{name}
```

## 🎮 Web UI

Navigate to `/services` page in your browser to:
- View all services and their status
- Start/Stop/Restart services with buttons
- View service logs
- Monitor health status
- See CPU/RAM usage

## ⚙️ Service Registry Format

```json
{
  "name": "service_name",
  "display_name": "Display Name",
  "type": "python|go|docker",
  "cmd": "command_to_run",
  "args": ["arg1", "arg2"],
  "working_dir": "relative/path",
  "autostart": true,
  "restart_policy": "never|on-failure|always",
  "health_check": "check_function_name",
  "port": 8081,
  "description": "Service description"
}
```

### Restart Policies

- **`never`**: Don't restart on crash
- **`on-failure`**: Restart only if service crashes (max 3 times)
- **`always`**: Always restart on crash (with cooldown)

## 🔍 Health Checks

Health checks are defined in `health_checks.py`. Available checks:

- `movement_check` - Checks Movement WebSocket server port
- `video_check` - Checks Video server port
- `api_check` - Checks Main API server port
- `ultrasonic_check` - Checks Ultrasonic server port
- `line_tracker_check` - Checks Line tracker server port
- `gateway_check` - Checks Gateway API port
- `ros_core_check` - Checks ROS 2 availability

Add custom health checks by:
1. Adding function to `health_checks.py`
2. Registering in `HEALTH_CHECKS` dict
3. Referencing in `service_registry.json`

## 📊 Monitoring

The orchestrator monitors:
- Process status (running/stopped/crashed)
- CPU usage per service
- Memory usage per service
- Crash count
- Last restart time
- Health check results

## 🛠️ Troubleshooting

### Service Won't Start

1. Check logs: `tail -f omega_services/logs/{service}.stderr.log`
2. Verify command path in registry
3. Check permissions
4. Test command manually

### Service Keeps Crashing

1. Check crash count in UI
2. Review logs for errors
3. Verify health check
4. Check restart policy
5. Review cooldown period (30s default)

### Orchestrator Not Starting

1. Check systemd logs: `sudo journalctl -u omega-orchestrator`
2. Verify Python path: `which python3`
3. Check file permissions
4. Verify service registry JSON is valid

## 🔒 Security

- Service runs as `omega1` user (not root)
- Logs stored in `omega_services/logs/`
- Process isolation
- No privilege escalation

## 📝 Logs

Service logs are stored in:
- `omega_services/logs/{service}.stdout.log`
- `omega_services/logs/{service}.stderr.log`
- Orchestrator logs: `omega_services/logs/services.log`
- Systemd logs: `sudo journalctl -u omega-orchestrator`

## 🚀 Future Enhancements

- [ ] Service dependencies (start B after A)
- [ ] Resource limits (CPU/RAM caps)
- [ ] Cloud-based service registry sync
- [ ] Distributed orchestrator (multi-robot)
- [ ] Event-driven notifications
- [ ] Service metrics export (Prometheus)

## 📚 Integration

The orchestrator integrates with:
- **Network Wizard** - Network services
- **ROS 2** - ROS node management
- **Main API** - Service control endpoints
- **Frontend UI** - Service management page

## 🎯 Usage Examples

### Start All Autostart Services
```bash
curl -X POST http://omega1.local:8080/api/services/start/movement_ws_server
```

### Check Service Health
```bash
curl http://omega1.local:8080/api/services/health/video_server
```

### View Service Logs
```bash
curl http://omega1.local:8080/api/services/logs/main_api?lines=100
```

### Restart Service
```bash
curl -X POST http://omega1.local:8080/api/services/restart/movement_ws_server
```

## 📖 Architecture

```
Service Manager (service_manager.py)
    ↓
Process Supervisor (process_supervisor.py)
    ↓
    ├─→ Process Monitoring (every 2s)
    ├─→ Health Checks (every 10s)
    ├─→ Auto-Restart (on crash)
    └─→ Log Management
```

## ✅ Status

**Current Implementation:** ✅ Complete
- Service registry ✅
- Process supervisor ✅
- Health checks ✅
- REST API ✅
- Web UI ✅
- Systemd service ✅

**Ready for Production:** ✅ Yes

