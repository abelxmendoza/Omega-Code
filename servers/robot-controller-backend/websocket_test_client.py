import asyncio
import websockets
import json
import ssl

async def connect():
    uri = "wss://localhost:8080/ws"
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.verify_mode = ssl.CERT_REQUIRED  # Enable certificate verification
    ssl_context.check_hostname = False  # Disable hostname check
    async with websockets.connect(uri, ssl=ssl_context) as websocket:
        message = {"message": "Hello, server!"}
        await websocket.send(json.dumps(message))
        response = await websocket.recv()
        print(f"Received: {response}")

asyncio.get_event_loop().run_until_complete(connect())