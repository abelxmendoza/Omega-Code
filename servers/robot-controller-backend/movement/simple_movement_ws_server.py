#!/usr/bin/env python3
import asyncio
import json
import time
import websockets
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class MovementWebSocketServer:
    def __init__(self, host='0.0.0.0', port=8081):
        self.host = host
        self.port = port
        self.clients = set()
        
    async def handle_client(self, websocket, path):
        self.clients.add(websocket)
        logger.info(f'Client connected: {websocket.remote_address}')
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                'type': 'welcome',
                'message': 'Connected to Movement WebSocket Server',
                'timestamp': int(time.time() * 1000)
            }))
            
            # Keep connection alive
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if data.get('type') == 'ping':
                        await websocket.send(json.dumps({
                            'type': 'pong',
                            'timestamp': int(time.time() * 1000)
                        }))
                except json.JSONDecodeError:
                    pass
                    
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.remove(websocket)
            logger.info(f'Client disconnected: {websocket.remote_address}')
    
    async def broadcast_data(self, data):
        if self.clients:
            message = json.dumps(data)
            await asyncio.gather(
                *[client.send(message) for client in self.clients],
                return_exceptions=True
            )
    
    async def start(self):
        logger.info(f'Starting Movement WebSocket server on {self.host}:{self.port}')
        async with websockets.serve(self.handle_client, self.host, self.port):
            await asyncio.Future()  # Run forever

if __name__ == '__main__':
    server = MovementWebSocketServer()
    asyncio.run(server.start())
