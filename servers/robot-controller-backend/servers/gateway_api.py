# servers/gateway_api.py
import os, json, time, asyncio
from typing import Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse, StreamingResponse, JSONResponse

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
    # Simple placeholder stream to keep UI happy until your video server is wired in.
    # When ready, replace this with a proper proxy to VIDEO_UPSTREAM.
    async def gen():
        boundary = b"--frame\r\n"
        while True:
            yield boundary + b"Content-Type: text/plain\r\n\r\nkeepalive\r\n"
            await asyncio.sleep(1.0)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

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
                await ws.send_text(json.dumps({
                    "type":"net-info",
                    "ssid":"OmegaNet", "ip":"192.168.1.123",
                    "iface":"wlan0", "tailscaleIp":"100.x.y.z",
                    "ts": int(time.time()*1000)
                }))
            elif cmd == "net-scan":
                await ws.send_text(json.dumps({
                    "type":"net-scan",
                    "networks":[
                        {"ssid":"OmegaNet","rssi":-54,"secure":True},
                        {"ssid":"Shop","rssi":-70,"secure":True},
                        {"ssid":"OpenGuest","rssi":-60,"secure":False},
                    ]
                }))
            elif cmd == "net-join":
                await ws.send_text(json.dumps({"type":"ack","status":"ok","action":"net-join"}))
            elif cmd == "net-forget":
                await ws.send_text(json.dumps({"type":"ack","status":"ok","action":"net-forget"}))
            else:
                # generic echo (lets pills measure latency)
                await ws.send_text(json.dumps({"type":"echo","data":data,"ts":time.time()}))
    except WebSocketDisconnect:
        return

@app.websocket("/ws/movement")
async def ws_movement(ws: WebSocket):   await ws_echo_with_wizard(ws, "movement")

@app.websocket("/ws/lighting")
async def ws_lighting(ws: WebSocket):   await ws_echo_with_wizard(ws, "lighting")

@app.websocket("/ws/ultrasonic")
async def ws_ultra(ws: WebSocket):      await ws_echo_with_wizard(ws, "ultrasonic")

@app.websocket("/ws/line")
async def ws_line(ws: WebSocket):       await ws_echo_with_wizard(ws, "line")
