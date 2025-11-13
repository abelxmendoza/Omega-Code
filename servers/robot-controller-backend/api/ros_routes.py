# api/ros_routes.py
"""
ROS 2 Docker Management API Routes

Provides endpoints for managing ROS 2 Docker containers:
- Start/stop containers
- Check container status
- View logs
- Monitor ROS 2 topics
"""
import os
import subprocess
import json
import logging
from typing import Optional, Dict, List
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel

log = logging.getLogger(__name__)

# Try to import native ROS2 bridge (optional - not available on macOS)
try:
    from .ros_native_bridge import get_bridge, init_ros2_bridge, _rclpy_available
    NATIVE_ROS2_AVAILABLE = _rclpy_available
except (ImportError, ModuleNotFoundError):
    NATIVE_ROS2_AVAILABLE = False
    get_bridge = None
    init_ros2_bridge = None

router = APIRouter(prefix="/api/ros", tags=["ROS 2"])

# Configuration - Support both laptop and Pi
# Auto-detect project root (works from laptop or Pi)
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ROS_DOCKER_COMPOSE_PATH = os.getenv(
    "ROS_DOCKER_COMPOSE_PATH", 
    os.path.join(_PROJECT_ROOT, "docker/ros2_robot/docker-compose.yml")
)
ROS_WORKSPACE_PATH = os.getenv("ROS_WORKSPACE_PATH", os.path.expanduser("~/omega_ws"))
ROS_NATIVE_MODE = os.getenv("ROS_NATIVE_MODE", "false").lower() == "true"  # Use native ROS2 instead of Docker

# Pi Docker configuration (for Raspberry Pi OS with Docker)
PI_SSH_HOST = os.getenv("PI_SSH_HOST", None)  # e.g., "pi@192.168.1.107"
PI_SSH_KEY = os.getenv("PI_SSH_KEY", None)  # Path to SSH key
PI_DOCKER_COMPOSE_PATH = os.getenv("PI_DOCKER_COMPOSE_PATH", "/home/pi/Omega-Code/docker/ros2_robot/docker-compose.yml")

CONTAINER_PREFIX = "omega_robot"

# Helper functions
def run_command(cmd: List[str], cwd: Optional[str] = None, timeout: int = 10, remote_host: Optional[str] = None) -> Dict[str, any]:
    """
    Run a shell command and return result
    If remote_host is provided, run command via SSH on remote host (Pi)
    """
    try:
        # If remote host specified, wrap command in SSH
        if remote_host:
            ssh_cmd = ["ssh"]
            if PI_SSH_KEY:
                ssh_cmd.extend(["-i", PI_SSH_KEY])
            ssh_cmd.extend(["-o", "StrictHostKeyChecking=no", remote_host])
            # Join command parts for remote execution
            remote_cmd = " ".join(f"'{part}'" for part in cmd)
            ssh_cmd.append(remote_cmd)
            cmd = ssh_cmd
        
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            cwd=cwd
        )
        return {
            "success": result.returncode == 0,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "returncode": result.returncode
        }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "stdout": "",
            "stderr": "Command timed out",
            "returncode": -1
        }
    except Exception as e:
        return {
            "success": False,
            "stdout": "",
            "stderr": str(e),
            "returncode": -1
        }

def get_container_status(service_name: str, remote_host: Optional[str] = None) -> Dict[str, any]:
    """Get status of a specific Docker container (local or remote Pi)"""
    result = run_command([
        "docker", "ps", "--filter", f"name={service_name}",
        "--format", "{{.Names}}\t{{.Status}}\t{{.ID}}"
    ], remote_host=remote_host)
    
    if result["success"] and result["stdout"].strip():
        parts = result["stdout"].strip().split("\t")
        return {
            "name": parts[0] if len(parts) > 0 else service_name,
            "status": parts[1] if len(parts) > 1 else "unknown",
            "id": parts[2] if len(parts) > 2 else "",
            "running": True
        }
    return {"name": service_name, "status": "stopped", "running": False}

# Models
class ContainerAction(BaseModel):
    action: str  # "start", "stop", "restart"
    service: Optional[str] = None  # "telemetry_publisher", "telemetry_listener", or None for all

# Routes
@router.get("/status")
def get_ros_status():
    """
    Get status of all ROS 2 containers (local or remote Pi)
    
    Works on:
    - Lenovo (Ubuntu): Native ROS2 + Pi Docker
    - MacBook (macOS): Pi Docker only (no native ROS2)
    - Pi: Local Docker only
    """
    try:
        # Determine if we should check Pi or local
        check_pi = PI_SSH_HOST is not None
        
        if check_pi:
            # Check Pi Docker containers via SSH
            result = run_command([
                "docker-compose", "-f", PI_DOCKER_COMPOSE_PATH, "ps", "--format", "json"
            ], remote_host=PI_SSH_HOST)
        else:
            # Check local Docker containers
            result = run_command([
                "docker-compose", "-f", ROS_DOCKER_COMPOSE_PATH, "ps", "--format", "json"
            ])
        
        containers = []
        if result["success"]:
            try:
                # Parse JSON output
                for line in result["stdout"].strip().split("\n"):
                    if line.strip():
                        containers.append(json.loads(line))
            except:
                pass
        
        # Fallback: check individual containers
        if not containers:
            if check_pi:
                # Check Pi containers via SSH
                publisher = get_container_status(f"{CONTAINER_PREFIX}_telemetry_publisher", remote_host=PI_SSH_HOST)
                listener = get_container_status(f"{CONTAINER_PREFIX}_telemetry_listener", remote_host=PI_SSH_HOST)
            else:
                publisher = get_container_status(f"{CONTAINER_PREFIX}_telemetry_publisher")
                listener = get_container_status(f"{CONTAINER_PREFIX}_telemetry_listener")
            
            containers = [
                {"Name": publisher["name"], "State": "running" if publisher["running"] else "stopped", "Status": publisher["status"]},
                {"Name": listener["name"], "State": "running" if listener["running"] else "stopped", "Status": listener["status"]}
            ]
        
        # Check ROS topics (try native first, then Docker)
        topics = []
        mode = "none"
        
        # Try native ROS2 (Lenovo laptop with ROS2)
        if ROS_NATIVE_MODE and NATIVE_ROS2_AVAILABLE:
            try:
                # Check if ROS2 is available (not on macOS)
                topic_result = run_command([
                    "bash", "-c", 
                    "source /opt/ros/rolling/setup.bash 2>/dev/null && ros2 topic list"
                ], timeout=5)
                if topic_result["success"]:
                    topics = [t.strip() for t in topic_result["stdout"].strip().split("\n") if t.strip() and not t.startswith("/")]
                    mode = "native"
            except:
                pass
        
        # Try Docker containers (local or Pi)
        if not topics:
            try:
                container_name = f"{CONTAINER_PREFIX}_telemetry_publisher_1"
                if check_pi:
                    # Execute in Pi container via SSH
                    topic_result = run_command([
                        "docker", "exec", "-i", container_name,
                        "bash", "-c", "source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash && ros2 topic list"
                    ], remote_host=PI_SSH_HOST, timeout=5)
                else:
                    topic_result = run_command([
                        "docker", "exec", "-i", container_name,
                        "bash", "-c", "source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash && ros2 topic list"
                    ], timeout=5)
                
                if topic_result["success"]:
                    topics = [t.strip() for t in topic_result["stdout"].strip().split("\n") if t.strip()]
                    mode = "docker" + ("_pi" if check_pi else "_local")
            except:
                pass
        
        compose_path = PI_DOCKER_COMPOSE_PATH if check_pi else ROS_DOCKER_COMPOSE_PATH
        
        return JSONResponse({
            "containers": containers,
            "topics": topics,
            "mode": mode,
            "location": "pi" if check_pi else "local",
            "compose_path": compose_path
        })
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/control")
def control_ros_container(action: ContainerAction):
    """Start, stop, or restart ROS 2 containers (local or remote Pi)"""
    try:
        # Determine if we should control Pi or local
        control_pi = PI_SSH_HOST is not None
        compose_path = PI_DOCKER_COMPOSE_PATH if control_pi else ROS_DOCKER_COMPOSE_PATH
        workspace_path = "/home/pi/Omega-Code" if control_pi else ROS_WORKSPACE_PATH
        
        cmd = ["docker-compose", "-f", compose_path]
        
        if action.action == "start":
            cmd.append("up")
            cmd.append("-d")
            if action.service:
                cmd.append(action.service)
        elif action.action == "stop":
            cmd.append("stop")
            if action.service:
                cmd.append(action.service)
        elif action.action == "restart":
            cmd.append("restart")
            if action.service:
                cmd.append(action.service)
        else:
            raise HTTPException(status_code=400, detail=f"Invalid action: {action.action}")
        
        result = run_command(cmd, cwd=workspace_path, timeout=30, remote_host=PI_SSH_HOST if control_pi else None)
        
        if result["success"]:
            location = "Pi" if control_pi else "local"
            return JSONResponse({
                "success": True,
                "message": f"Container {action.service or 'all'} {action.action}ed successfully on {location}",
                "output": result["stdout"],
                "location": location
            })
        else:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to {action.action} container: {result['stderr']}"
            )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/logs/{service}")
def get_ros_logs(service: str, tail: int = 50):
    """Get logs from a ROS 2 container"""
    try:
        container_name = f"{CONTAINER_PREFIX}_{service}_1"
        result = run_command([
            "docker", "logs", "--tail", str(tail), container_name
        ], timeout=5)
        
        if result["success"]:
            return JSONResponse({
                "service": service,
                "logs": result["stdout"].split("\n")
            })
        else:
            # Try alternative container name pattern
            result = run_command([
                "docker", "logs", "--tail", str(tail), f"{service}_1"
            ], timeout=5)
            if result["success"]:
                return JSONResponse({
                    "service": service,
                    "logs": result["stdout"].split("\n")
                })
            raise HTTPException(status_code=404, detail=f"Container {service} not found")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/topics")
def list_ros_topics():
    """List available ROS 2 topics"""
    try:
        # Try native ROS2 first if available
        if ROS_NATIVE_MODE and NATIVE_ROS2_AVAILABLE:
            try:
                result = run_command([
                    "bash", "-c", 
                    "source /opt/ros/rolling/setup.bash 2>/dev/null && ros2 topic list"
                ], timeout=5)
                if result["success"]:
                    topics = [t.strip() for t in result["stdout"].strip().split("\n") if t.strip() and not t.startswith("/")]
                    return JSONResponse({"topics": topics, "source": "native", "mode": "native"})
            except Exception as e:
                pass  # Fall back to Docker
        
        # Try Docker containers
        containers = ["telemetry_publisher", "telemetry_listener"]
        for service in containers:
            container_name = f"{CONTAINER_PREFIX}_{service}_1"
            result = run_command([
                "docker", "exec", "-i", container_name,
                "bash", "-c", "source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash && ros2 topic list"
            ], timeout=5)
            
            if result["success"]:
                topics = [t.strip() for t in result["stdout"].strip().split("\n") if t.strip()]
                return JSONResponse({"topics": topics, "source": service, "mode": "docker"})
        
        return JSONResponse({"topics": [], "message": "No ROS2 connection available", "mode": "none"})
    except Exception as e:
        return JSONResponse({"topics": [], "error": str(e)})

@router.websocket("/bridge")
async def ros2_web_bridge_websocket(websocket: WebSocket):
    """
    ROS2-Web Bridge WebSocket
    
    Real-time bidirectional bridge between ROS2 topics and web app.
    Subscribe to ROS2 topics and receive messages in real-time.
    Publish commands from web app to ROS2 topics.
    
    Message format:
    {
        "type": "subscribe|publish|unsubscribe|list_topics",
        "topic": "/omega/sensors/ultrasonic",
        "msg_type": "String|Float32|Twist|BatteryState",
        "command": {...}  // for publish
    }
    """
    from .ros_web_bridge import get_ros2_web_bridge
    
    bridge = get_ros2_web_bridge()
    if not bridge:
        await websocket.accept()
        await websocket.send_json({
            "error": "ROS2 not available",
            "message": "ROS2 bridge is not initialized. Enable ROS_NATIVE_MODE on Lenovo or connect to Pi Docker."
        })
        await websocket.close()
        return
    
    await bridge.connect(websocket)
    
    try:
        while True:
            data = await websocket.receive_json()
            await bridge.handle_websocket_message(websocket, data)
    except WebSocketDisconnect:
        bridge.disconnect(websocket)
    except Exception as e:
        log.error(f"WebSocket error: {e}")
        bridge.disconnect(websocket)

@router.websocket("/telemetry")
async def ros_telemetry_websocket(websocket: WebSocket):
    """WebSocket bridge for ROS 2 telemetry topic"""
    await websocket.accept()
    
    try:
        # Find running container
        container_name = None
        for service in ["telemetry_publisher", "telemetry_listener"]:
            result = run_command([
                "docker", "ps", "--filter", f"name={CONTAINER_PREFIX}_{service}", "--format", "{{.Names}}"
            ], timeout=3)
            if result["success"] and result["stdout"].strip():
                container_name = result["stdout"].strip().split("\n")[0]
                break
        
        if not container_name:
            await websocket.send_json({"error": "No ROS containers running"})
            await websocket.close()
            return
        
        # Start ros2 topic echo in container
        process = subprocess.Popen(
            [
                "docker", "exec", "-i", container_name,
                "bash", "-c",
                "source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash && ros2 topic echo /omega/telemetry --once"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Poll for messages
        import asyncio
        while True:
            await asyncio.sleep(1)
            
            # Check if process is still running
            if process.poll() is not None:
                # Process finished, restart it
                process = subprocess.Popen(
                    [
                        "docker", "exec", "-i", container_name,
                        "bash", "-c",
                        "source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash && timeout 5 ros2 topic echo /omega/telemetry"
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
            
            # Try to read output
            try:
                line = process.stdout.readline()
                if line:
                    await websocket.send_json({"topic": "/omega/telemetry", "data": line.strip()})
            except:
                pass
            
            # Keep connection alive
            try:
                await websocket.receive_text()
            except:
                pass
                
    except WebSocketDisconnect:
        pass
    except Exception as e:
        try:
            await websocket.send_json({"error": str(e)})
        except:
            pass

