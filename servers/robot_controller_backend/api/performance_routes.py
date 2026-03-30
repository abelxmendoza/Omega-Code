"""
Performance API Routes — real-time system metrics from the Pi.
"""

import time
import socket
import subprocess
import logging
from fastapi import APIRouter
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/performance", tags=["Performance"])


def _cpu_temperature() -> float:
    try:
        result = subprocess.run(
            ["vcgencmd", "measure_temp"], capture_output=True, text=True, timeout=2
        )
        if result.returncode == 0:
            # output: "temp=45.1'C"
            return float(result.stdout.split("=")[1].split("'")[0])
    except Exception:
        pass
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:
        return 0.0


@router.get("/metrics")
async def get_performance_metrics():
    """Get real-time system performance metrics."""
    try:
        import psutil

        cpu_percent = psutil.cpu_percent(interval=0.2)
        memory = psutil.virtual_memory()
        disk = psutil.disk_usage("/")
        net_io = psutil.net_io_counters()
        connections = len(psutil.net_connections())
        uptime = time.time() - psutil.boot_time()
        load_avg = psutil.getloadavg()[0] if hasattr(psutil, "getloadavg") else 0.0

        return JSONResponse(
            content={
                "timestamp": int(time.time() * 1000),
                "source": "Raspberry Pi",
                "deviceName": socket.gethostname(),
                "cpuUsage": cpu_percent,
                "memoryUsage": memory.used,
                "memoryPercent": memory.percent,
                "memoryTotal": memory.total,
                "diskUsage": disk.percent,
                "diskUsed": disk.used,
                "diskTotal": disk.total,
                "networkIO": {
                    "bytesSent": net_io.bytes_sent,
                    "bytesRecv": net_io.bytes_recv,
                    "packetsSent": net_io.packets_sent,
                    "packetsRecv": net_io.packets_recv,
                },
                "websocketConnections": connections,
                "uptime": uptime,
                "loadAverage": load_avg,
                "temperature": _cpu_temperature(),
            }
        )
    except ImportError:
        return JSONResponse(
            content={"error": "psutil not installed"}, status_code=503
        )
    except Exception as e:
        logger.error("performance metrics error: %s", e)
        return JSONResponse(
            content={"error": f"Failed to get metrics: {e}"}, status_code=500
        )


@router.get("/cache")
async def get_cache_stats():
    """Get cache statistics (stub — no cache layer currently)."""
    return JSONResponse(
        content={
            "hits": 0,
            "misses": 0,
            "hitRate": 0.0,
            "totalRequests": 0,
            "cacheSize": 0,
            "maxSize": 1000,
        }
    )
