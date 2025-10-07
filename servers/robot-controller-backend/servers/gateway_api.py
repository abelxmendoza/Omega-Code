# servers/gateway_api.py
import os, json, time, asyncio
from typing import Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse, StreamingResponse, JSONResponse
import websockets

# ---------- Config ----------
PORT = int(os.getenv("PORT", "7070"))

# Optional downstreams (set these later to proxy through to your real services)
DS_MOVE = os.getenv("DS_MOVE_WS")          # e.g. ws://127.0.0.1:8081
DS_LINE = os.getenv("DS_LINE_WS")          # e.g. ws://127.0.0.1:8090/line-tracker
DS_LIGHT= os.getenv("DS_LIGHT_WS")         # e.g. ws://127.0.0.1:8082/lighting
DS_ULTRA= os.getenv("DS_ULTRA_WS")         # e.g. ws://127.0.0.1:8080/ultrasonic
VIDEO_UPSTREAM = os.getenv("VIDEO_UPSTREAM", "http://127.0.0.1:5000/video_feed")

app = FastAPI(title="Omega Gateway")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_methods=["*"], allow_headers=["*"], allow_credentials=True,
)

# ---------- Health + Video (/health used by header) ----------
@app.get("/health", response_class=PlainTextResponse)
def health():
    return "ok"

@app.get("/video_feed")
def video_feed():
    # Proxy video feed from the actual video server
    import httpx
    
    async def proxy_video():
        async with httpx.AsyncClient() as client:
            async with client.stream("GET", VIDEO_UPSTREAM) as response:
                async for chunk in response.aiter_bytes():
                    yield chunk
    
    return StreamingResponse(proxy_video(), media_type="multipart/x-mixed-replace; boundary=frame")

# ---------- Network endpoints used by the Header + Wizard ----------
@app.get("/api/net/summary")
def net_summary():
    # Basic, static summary so your Live Link pill works immediately.
    # Replace with real data when you have it.
    return {
        "linkType": os.getenv("LINK_TYPE", "wifi"),
        "online": True,
        "ssid": os.getenv("SSID", "OmegaNet"),
        "ifname": "wlan0",
        "ipv4": "192.168.1.123",
        "gateway": "192.168.1.1",
        "rssi": -55,
    }

@app.post("/api/net/pan/connect")
async def connect_pan(payload: dict):
    # TODO: call your real helper (shell script / python / go) here
    return {"message": "Requested PAN connect"}

@app.post("/api/net/wifi/connect")
async def connect_wifi(payload: dict):
    ssid = payload.get("ssid", "")
    # TODO: call your real helper here (nmcli/iw/systemd service)
    return {"message": f"Requested Wi-Fi connect to \"{ssid}\""}

# ---------- WebSocket helpers ----------
async def ws_proxy_to_downstream(ws: WebSocket, downstream_url: str, role: str):
    """Proxy WebSocket connection to downstream service."""
    if not downstream_url:
        print(f"[GATEWAY] No downstream URL configured for {role}, using echo mode")
        await ws_echo_with_wizard(ws, role)
        return
    
    print(f"[GATEWAY] Proxying {role} to {downstream_url}")
    await ws.accept()
    
    try:
        async with websockets.connect(downstream_url) as downstream_ws:
            print(f"[GATEWAY] Connected to downstream {role} service")
            
            # Create tasks for bidirectional message forwarding
            async def forward_to_downstream():
                try:
                    while True:
                        message = await ws.receive_text()
                        await downstream_ws.send(message)
                except WebSocketDisconnect:
                    pass
            
            async def forward_to_client():
                try:
                    async for message in downstream_ws:
                        await ws.send_text(message)
                except WebSocketDisconnect:
                    pass
            
            # Run both forwarding tasks concurrently
            await asyncio.gather(
                forward_to_downstream(),
                forward_to_client(),
                return_exceptions=True
            )
            
    except Exception as e:
        print(f"[GATEWAY] Error connecting to downstream {role}: {e}")
        await ws.close(code=1011, reason="Downstream service unavailable")
    except WebSocketDisconnect:
        print(f"[GATEWAY] Client disconnected from {role}")

async def ws_echo_with_wizard(ws: WebSocket, role: str):
    """A working stub so UI pills & the Network Wizard function now."""
    await ws.accept()
    try:
        while True:
            raw = await ws.receive_text()
            try:
                data = json.loads(raw)
            except Exception:
                data = {"raw": raw}

            cmd = (isinstance(data, dict) and data.get("command")) or None
            if cmd == "keep-alive":
                await ws.send_text(json.dumps({"type": "pong", "ts": time.time()}))
            elif cmd == "net-info":
                # Get current network information
                try:
                    import subprocess
                    import socket
                    
                    # Get current SSID
                    ssid = "Unknown"
                    try:
                        result = subprocess.run(['iwgetid', '-r'], capture_output=True, text=True, timeout=5)
                        if result.returncode == 0 and result.stdout.strip():
                            ssid = result.stdout.strip()
                    except Exception:
                        pass
                    
                    # Get IP address
                    ip = "Unknown"
                    try:
                        # Get primary network interface IP
                        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=5)
                        if result.returncode == 0 and result.stdout.strip():
                            ip = result.stdout.strip().split()[0]  # First IP
                    except Exception:
                        try:
                            # Fallback: get IP of default interface
                            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                            s.connect(("8.8.8.8", 80))
                            ip = s.getsockname()[0]
                            s.close()
                        except Exception:
                            pass
                    
                    # Get network interface
                    iface = "Unknown"
                    try:
                        result = subprocess.run(['ip', 'route', 'get', '8.8.8.8'], capture_output=True, text=True, timeout=5)
                        if result.returncode == 0:
                            for line in result.stdout.split('\n'):
                                if 'dev' in line:
                                    iface = line.split('dev')[1].split()[0]
                                    break
                    except Exception:
                        pass
                    
                    # Check for Tailscale IP
                    tailscale_ip = None
                    try:
                        result = subprocess.run(['ip', 'addr', 'show', 'tailscale0'], capture_output=True, text=True, timeout=5)
                        if result.returncode == 0:
                            for line in result.stdout.split('\n'):
                                if 'inet' in line and 'tailscale0' in line:
                                    tailscale_ip = line.split()[1].split('/')[0]
                                    break
                    except Exception:
                        pass
                    
                    await ws.send_text(json.dumps({
                        "type": "net-info",
                        "ssid": ssid,
                        "ip": ip,
                        "iface": iface,
                        "tailscaleIp": tailscale_ip,
                        "ts": int(time.time()*1000)
                    }))
                except Exception as e:
                    await ws.send_text(json.dumps({"type":"ack","status":"error","error":f"net-info-failed: {str(e)}"}))

            elif cmd == "net-scan":
                # Scan for nearby WiFi networks
                try:
                    import subprocess
                    networks = []
                    
                    # Use iwlist to scan for networks
                    result = subprocess.run(['iwlist', 'scan'], capture_output=True, text=True, timeout=15)
                    if result.returncode == 0:
                        current_ssid = None
                        current_rssi = None
                        current_secure = False
                        
                        for line in result.stdout.split('\n'):
                            line = line.strip()
                            if 'ESSID:' in line:
                                if current_ssid:
                                    networks.append({
                                        "ssid": current_ssid,
                                        "rssi": current_rssi,
                                        "secure": current_secure
                                    })
                                current_ssid = line.split('ESSID:')[1].strip().strip('"')
                                if current_ssid == '':
                                    current_ssid = '(hidden)'
                            elif 'Signal level=' in line:
                                try:
                                    rssi_str = line.split('Signal level=')[1].split()[0]
                                    current_rssi = int(rssi_str)
                                except Exception:
                                    current_rssi = None
                            elif 'Encryption key:on' in line:
                                current_secure = True
                            elif 'Encryption key:off' in line:
                                current_secure = False
                        
                        # Add the last network
                        if current_ssid:
                            networks.append({
                                "ssid": current_ssid,
                                "rssi": current_rssi,
                                "secure": current_secure
                            })
                    
                    await ws.send_text(json.dumps({
                        "type": "net-scan",
                        "networks": networks,
                        "ts": int(time.time()*1000)
                    }))
                except Exception as e:
                    await ws.send_text(json.dumps({"type":"ack","status":"error","error":f"net-scan-failed: {str(e)}"}))

            elif cmd == "net-join":
                # Join a WiFi network
                ssid = data.get("ssid", "").strip()
                password = data.get("pass", "").strip()
                
                if not ssid:
                    await ws.send_text(json.dumps({"type":"ack","status":"error","error":"SSID required"}))
                else:
                    try:
                        import subprocess
                        
                        # Try to connect using nmcli (NetworkManager)
                        if password:
                            result = subprocess.run([
                                'nmcli', 'device', 'wifi', 'connect', ssid, 'password', password
                            ], capture_output=True, text=True, timeout=30)
                        else:
                            result = subprocess.run([
                                'nmcli', 'device', 'wifi', 'connect', ssid
                            ], capture_output=True, text=True, timeout=30)
                        
                        if result.returncode == 0:
                            await ws.send_text(json.dumps({"type":"ack","status":"ok","action":"net-join","ssid":ssid}))
                        else:
                            await ws.send_text(json.dumps({"type":"ack","status":"error","error":result.stderr.strip()}))
                            
                    except Exception as e:
                        await ws.send_text(json.dumps({"type":"ack","status":"error","error":f"net-join-failed: {str(e)}"}))

            elif cmd == "net-forget":
                # Forget a WiFi network
                ssid = data.get("ssid", "").strip()
                
                if not ssid:
                    await ws.send_text(json.dumps({"type":"ack","status":"error","error":"SSID required"}))
                else:
                    try:
                        import subprocess
                        
                        # Use nmcli to forget the network
                        result = subprocess.run([
                            'nmcli', 'connection', 'delete', ssid
                        ], capture_output=True, text=True, timeout=10)
                        
                        if result.returncode == 0:
                            await ws.send_text(json.dumps({"type":"ack","status":"ok","action":"net-forget","ssid":ssid}))
                        else:
                            # Try alternative method
                            result2 = subprocess.run([
                                'nmcli', 'connection', 'delete', f'"{ssid}"'
                            ], capture_output=True, text=True, timeout=10)
                            
                            if result2.returncode == 0:
                                await ws.send_text(json.dumps({"type":"ack","status":"ok","action":"net-forget","ssid":ssid}))
                            else:
                                await ws.send_text(json.dumps({"type":"ack","status":"error","error":result.stderr.strip()}))
                                
                    except Exception as e:
                        await ws.send_text(json.dumps({"type":"ack","status":"error","error":f"net-forget-failed: {str(e)}"}))
            else:
                # generic echo (lets pills measure latency)
                await ws.send_text(json.dumps({"type":"echo","data":data,"ts":time.time()}))
    except WebSocketDisconnect:
        return

@app.websocket("/ws/movement")
async def ws_movement(ws: WebSocket):
    await ws_proxy_to_downstream(ws, DS_MOVE, "movement")

@app.websocket("/ws/lighting")
async def ws_lighting(ws: WebSocket):
    await ws_proxy_to_downstream(ws, DS_LIGHT, "lighting")

@app.websocket("/ws/ultrasonic")
async def ws_ultra(ws: WebSocket):
    await ws_proxy_to_downstream(ws, DS_ULTRA, "ultrasonic")

@app.websocket("/ws/line")
async def ws_line(ws: WebSocket):
    await ws_proxy_to_downstream(ws, DS_LINE, "line")

# ---------- Performance API endpoints ----------
@app.get("/api/performance/metrics")
async def get_performance_metrics():
    """Proxy performance metrics from the Pi's performance API"""
    try:
        import httpx
        async with httpx.AsyncClient() as client:
            response = await client.get("http://127.0.0.1:8080/api/performance/metrics", timeout=5.0)
            return JSONResponse(content=response.json(), status_code=response.status_code)
    except Exception as e:
        return JSONResponse(content={"error": f"Performance API unavailable: {str(e)}"}, status_code=503)

@app.get("/api/performance/cache")
async def get_cache_stats():
    """Proxy cache statistics from the Pi's performance API"""
    try:
        import httpx
        async with httpx.AsyncClient() as client:
            response = await client.get("http://127.0.0.1:8080/api/performance/cache", timeout=5.0)
            return JSONResponse(content=response.json(), status_code=response.status_code)
    except Exception as e:
        return JSONResponse(content={"error": f"Cache API unavailable: {str(e)}"}, status_code=503)

@app.get("/api/performance/system")
async def get_system_info():
    """Proxy system information from the Pi's performance API"""
    try:
        import httpx
        async with httpx.AsyncClient() as client:
            response = await client.get("http://127.0.0.1:8080/api/performance/system", timeout=5.0)
            return JSONResponse(content=response.json(), status_code=response.status_code)
    except Exception as e:
        return JSONResponse(content={"error": f"System API unavailable: {str(e)}"}, status_code=503)
