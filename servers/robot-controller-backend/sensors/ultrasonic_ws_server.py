#!/usr/bin/env python3
# File: ultrasonic_ws_server.py
# Summary: Ultrasonic WebSocket Server for HC-SR04 sensor

import asyncio
import websockets
import json
import time
import logging
from ultrasonic_sensor import Ultrasonic

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class UltrasonicWebSocketServer:
    def __init__(self):
        self.ultrasonic = None
        self.clients = set()
        
    async def handle_client(self, websocket, path):
        """Handle WebSocket client connections"""
        logger.info(f'Client connected: {websocket.remote_address}')
        self.clients.add(websocket)
        
        try:
            # Send welcome message
            await websocket.send(json.dumps({
                'type': 'welcome',
                'message': 'Connected to Ultrasonic WebSocket Server',
                'service': 'ultrasonic',
                'timestamp': int(time.time() * 1000)
            }))
            
            # Initialize ultrasonic sensor
            if not self.ultrasonic:
                try:
                    self.ultrasonic = Ultrasonic()
                    logger.info('Ultrasonic sensor initialized')
                except Exception as e:
                    logger.error(f'Failed to initialize ultrasonic sensor: {e}')
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'message': f'Sensor initialization failed: {e}',
                        'timestamp': int(time.time() * 1000)
                    }))
                    return
            
            # Handle incoming messages
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
    
    async def send_distance_data(self):
        """Send distance data to all connected clients"""
        while True:
            if self.clients and self.ultrasonic:
                try:
                    distance = self.ultrasonic.get_distance()
                    
                    data = {
                        'type': 'ultrasonic',
                        'distance': distance,
                        'distance_cm': distance,
                        'distance_m': round(distance / 100.0, 2) if distance != -1 else 0,
                        'distance_inch': round(distance / 2.54, 2) if distance != -1 else 0,
                        'distance_feet': round(distance / 30.48, 2) if distance != -1 else 0,
                        'status': 'success' if distance != -1 else 'error',
                        'error': 'Sensor timeout - check wiring' if distance == -1 else None,
                        'timestamp': int(time.time() * 1000)
                    }
                    
                    # Send to all connected clients
                    disconnected = set()
                    for client in self.clients:
                        try:
                            await client.send(json.dumps(data))
                        except websockets.exceptions.ConnectionClosed:
                            disconnected.add(client)
                    
                    # Remove disconnected clients
                    self.clients -= disconnected
                    
                    if distance != -1:
                        logger.info(f'Distance: {distance} cm')
                    else:
                        logger.warning('Sensor timeout - check wiring')
                        
                except Exception as e:
                    logger.error(f'Error reading sensor: {e}')
                    # Send error to clients
                    error_data = {
                        'type': 'ultrasonic',
                        'distance': -1,
                        'status': 'error',
                        'error': str(e),
                        'timestamp': int(time.time() * 1000)
                    }
                    
                    disconnected = set()
                    for client in self.clients:
                        try:
                            await client.send(json.dumps(error_data))
                        except websockets.exceptions.ConnectionClosed:
                            disconnected.add(client)
                    self.clients -= disconnected
            
            await asyncio.sleep(1)  # Send data every second
    
    async def start_server(self, host='0.0.0.0', port=8080):
        """Start the WebSocket server"""
        logger.info(f'Starting Ultrasonic WebSocket server on {host}:{port}')
        
        # Start distance data sender
        asyncio.create_task(self.send_distance_data())
        
        # Start WebSocket server
        async with websockets.serve(self.handle_client, host, port, subprotocols=["json"]):
            await asyncio.Future()  # Run forever

async def main():
    server = UltrasonicWebSocketServer()
    await server.start_server()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info('Server stopped by user')
    except Exception as e:
        logger.error(f'Server error: {e}')
