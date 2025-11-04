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
from typing import Optional, Dict, List
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel

router = APIRouter(prefix="/api/ros", tags=["ROS 2"])

# Configuration
ROS_DOCKER_COMPOSE_PATH = os.getenv("ROS_DOCKER_COMPOSE_PATH", "/home/omega1/Omega-Code/docker/ros2_robot/docker-compose.yml")
ROS_WORKSPACE_PATH = os.getenv("ROS_WORKSPACE_PATH", "/home/omega1/Omega-Code")
CONTAINER_PREFIX = "omega_robot"

# Helper functions
def run_command(cmd: List[str], cwd: Optional[str] = None, timeout: int = 10) -> Dict[str, any]:
    """Run a shell command and return result"""
    try:
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

def get_container_status(service_name: str) -> Dict[str, any]:
    """Get status of a specific Docker container"""
    result = run_command([
        "docker", "ps", "--filter", f"name={service_name}",
        "--format", "{{.Names}}\t{{.Status}}\t{{.ID}}"
    ])
    
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
    """Get status of all ROS 2 containers"""
    try:
        # Check docker-compose services
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
            publisher = get_container_status(f"{CONTAINER_PREFIX}_telemetry_publisher")
            listener = get_container_status(f"{CONTAINER_PREFIX}_telemetry_listener")
            containers = [
                {"Name": publisher["name"], "State": "running" if publisher["running"] else "stopped", "Status": publisher["status"]},
                {"Name": listener["name"], "State": "running" if listener["running"] else "stopped", "Status": listener["status"]}
            ]
        
        # Check ROS topics (if any container is running)
        topics = []
        try:
            topic_result = run_command([
                "docker", "exec", "-i", f"{CONTAINER_PREFIX}_telemetry_publisher_1",
                "bash", "-c", "source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash && ros2 topic list"
            ], timeout=5)
            if topic_result["success"]:
                topics = [t.strip() for t in topic_result["stdout"].strip().split("\n") if t.strip()]
        except:
            pass
        
        return JSONResponse({
            "containers": containers,
            "topics": topics,
            "compose_path": ROS_DOCKER_COMPOSE_PATH
        })
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/control")
def control_ros_container(action: ContainerAction):
    """Start, stop, or restart ROS 2 containers"""
    try:
        cmd = ["docker-compose", "-f", ROS_DOCKER_COMPOSE_PATH]
        
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
        
        result = run_command(cmd, cwd=ROS_WORKSPACE_PATH, timeout=30)
        
        if result["success"]:
            return JSONResponse({
                "success": True,
                "message": f"Container {action.service or 'all'} {action.action}ed successfully",
                "output": result["stdout"]
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
        # Try to exec into any running container
        containers = ["telemetry_publisher", "telemetry_listener"]
        for service in containers:
            container_name = f"{CONTAINER_PREFIX}_{service}_1"
            result = run_command([
                "docker", "exec", "-i", container_name,
                "bash", "-c", "source /opt/ros/humble/setup.bash && source /root/omega_ws/install/setup.bash && ros2 topic list"
            ], timeout=5)
            
            if result["success"]:
                topics = [t.strip() for t in result["stdout"].strip().split("\n") if t.strip()]
                return JSONResponse({"topics": topics, "source": service})
        
        return JSONResponse({"topics": [], "message": "No running containers found"})
    except Exception as e:
        return JSONResponse({"topics": [], "error": str(e)})

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

