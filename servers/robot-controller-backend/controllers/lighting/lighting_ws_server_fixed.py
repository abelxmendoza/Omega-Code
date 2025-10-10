#!/usr/bin/env python3
import asyncio
import json
import time
import websockets
import logging
import subprocess
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LightingWebSocketServer:
    def __init__(self, host='0.0.0.0', port=8082):
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
                'message': 'Connected to Lighting WebSocket Server',
                'timestamp': int(time.time() * 1000)
            }))
            
            # Keep connection alive and handle commands
            async for message in websocket:
                try:
                    data = json.loads(message)
                    
                    if data.get('type') == 'ping':
                        await websocket.send(json.dumps({
                            'type': 'pong',
                            'timestamp': int(time.time() * 1000)
                        }))
                    elif data.get('type') == 'lighting_command':
                        await self.handle_lighting_command(data, websocket)
                        
                except json.JSONDecodeError:
                    logger.warning('Invalid JSON received')
                    
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.remove(websocket)
            logger.info(f'Client disconnected: {websocket.remote_address}')
    
    async def handle_lighting_command(self, data, websocket):
        """Handle lighting control commands"""
        try:
            command = data.get('command', '')
            color = data.get('color', '#ffffff')
            
            logger.info(f'Received lighting command: {command}, color: {color}')
            
            # Convert hex color to RGB
            if color.startswith('#'):
                color = color[1:]
            
            # Map color names to RGB values
            color_map = {
                'red': '255 0 0',
                'green': '0 255 0', 
                'blue': '0 0 255',
                'white': '255 255 255',
                'off': '0 0 0',
                'yellow': '255 255 0',
                'purple': '255 0 255',
                'cyan': '0 255 255'
            }
            
            if color.lower() in color_map:
                rgb_values = color_map[color.lower()]
            else:
                # Try to parse hex color
                try:
                    r = int(color[0:2], 16)
                    g = int(color[2:4], 16) 
                    b = int(color[4:6], 16)
                    rgb_values = f'{r} {g} {b}'
                except:
                    rgb_values = '255 255 255'  # Default to white
            
            # Execute LED control command
            script_path = '/home/omega1/Omega-Code/servers/robot-controller-backend/controllers/lighting/rgb_led_test.py'
            cmd = ['sudo', 'python3', script_path] + rgb_values.split()
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                await websocket.send(json.dumps({
                    'type': 'lighting_response',
                    'status': 'success',
                    'command': command,
                    'color': color,
                    'message': f'LEDs set to {color}',
                    'timestamp': int(time.time() * 1000)
                }))
                logger.info(f'Successfully set LEDs to {color}')
            else:
                await websocket.send(json.dumps({
                    'type': 'lighting_response',
                    'status': 'error',
                    'command': command,
                    'color': color,
                    'message': f'Failed to set LEDs: {result.stderr}',
                    'timestamp': int(time.time() * 1000)
                }))
                logger.error(f'Failed to set LEDs: {result.stderr}')
                
        except Exception as e:
            logger.error(f'Error handling lighting command: {e}')
            await websocket.send(json.dumps({
                'type': 'lighting_response',
                'status': 'error',
                'message': str(e),
                'timestamp': int(time.time() * 1000)
            }))
    
    async def broadcast_data(self, data):
        if self.clients:
            message = json.dumps(data)
            await asyncio.gather(
                *[client.send(message) for client in self.clients],
                return_exceptions=True
            )
    
    async def start(self):
        logger.info(f'Starting Lighting WebSocket server on {self.host}:{self.port}')
        async with websockets.serve(self.handle_client, self.host, self.port):
            await asyncio.Future()  # Run forever

if __name__ == '__main__':
    server = LightingWebSocketServer()
    asyncio.run(server.start())
