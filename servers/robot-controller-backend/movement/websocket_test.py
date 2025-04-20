import websocket

ws = websocket.create_connection("ws://127.0.0.1:8081/movement")
ws.send("test")
print("WebSocket test successful!")
ws.close()
