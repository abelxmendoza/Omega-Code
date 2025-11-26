#!/usr/bin/env python3
# File: ultrasonic_ws_server.py
# Summary: Ultrasonic WebSocket Server for HC-SR04 sensor (Python Alternative)
#
# ⚠️ NOTE: This is a Python alternative to main_ultrasonic.go
# For production deployment, main_ultrasonic.go is recommended (optimized Go server)
# Use this file if you prefer Python or need Python-specific features
#
# Purpose: Python WebSocket server providing same functionality as main_ultrasonic.go
# Port: 8080 (same as Go version - cannot run both simultaneously)
# Dependencies: websockets, ultrasonic_sensor.py

import asyncio
import websockets
import json
import time
import logging
from collections import deque
from typing import Set, Optional
from ultrasonic_sensor import Ultrasonic

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Constants - cached for performance
CONVERSION_FACTORS = {
    'm': 100.0,
    'inch': 2.54,
    'feet': 30.48
}

class UltrasonicWebSocketServer:
    def __init__(self):
        self.ultrasonic: Optional[Ultrasonic] = None
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self._lock = asyncio.Lock()  # Thread-safe client management
        self._last_distance = -1  # Cache last reading for comparison
        
    async def handle_client(self, websocket, path):
        """Handle WebSocket client connections with optimized error handling"""
        logger.info(f'Client connected: {websocket.remote_address}')
        
        async with self._lock:
            self.clients.add(websocket)
        
        try:
            # Pre-serialize welcome message for efficiency
            welcome_msg = json.dumps({
                'status': 'connected',
                'service': 'ultrasonic',
                'message': 'Ultrasonic WebSocket connection established',
                'ts': int(time.time() * 1000)
            })
            await websocket.send(welcome_msg)
            
            # Initialize ultrasonic sensor (thread-safe)
            if not self.ultrasonic:
                try:
                    self.ultrasonic = Ultrasonic()
                    logger.info('Ultrasonic sensor initialized')
                except Exception as e:
                    logger.error(f'Failed to initialize ultrasonic sensor: {e}')
                    error_msg = json.dumps({
                        'status': 'error',
                        'error': f'Sensor initialization failed: {e}',
                        'ts': int(time.time() * 1000)
                    })
                    await websocket.send(error_msg)
                    return
            
            # Handle incoming messages with optimized ping/pong
            pong_template = json.dumps({'type': 'pong'})  # Pre-serialize
            
            async for message in websocket:
                try:
                    # Fast path: check if it's a ping without full JSON parse
                    if message.strip() == '{"type":"ping"}' or '"ping"' in message:
                        data = json.loads(message)
                        if data.get('type') == 'ping':
                            # Use pre-serialized template and add timestamp
                            pong_msg = json.dumps({
                                'type': 'pong',
                                'ts': data.get('ts', int(time.time() * 1000))
                            })
                            await websocket.send(pong_msg)
                except (json.JSONDecodeError, KeyError):
                    pass  # Ignore malformed messages
                    
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.error(f'Error handling client {websocket.remote_address}: {e}')
        finally:
            async with self._lock:
                self.clients.discard(websocket)  # Use discard instead of remove (safer)
            logger.info(f'Client disconnected: {websocket.remote_address}')
    
    async def send_distance_data(self):
        """Send distance data to all connected clients with optimized batching"""
        while True:
            # Check if we have clients and sensor (atomic check)
            async with self._lock:
                clients_copy = self.clients.copy() if self.clients else set()
            
            if clients_copy and self.ultrasonic:
                try:
                    distance = self.ultrasonic.get_distance()
                    timestamp = int(time.time() * 1000)
                    
                    # Pre-calculate conversions using cached constants
                    if distance != -1:
                        # Success case - pre-serialize once
                        data = {
                            'status': 'success',
                            'distance_cm': distance,
                            'distance_m': round(distance / CONVERSION_FACTORS['m'], 2),
                            'distance_inch': round(distance / CONVERSION_FACTORS['inch'], 2),
                            'distance_feet': round(distance / CONVERSION_FACTORS['feet'], 2),
                            'ts': timestamp
                        }
                        serialized_data = json.dumps(data)
                        
                        # Batch send to all clients (non-blocking)
                        disconnected = set()
                        send_tasks = []
                        
                        for client in clients_copy:
                            try:
                                send_tasks.append(client.send(serialized_data))
                            except Exception:
                                disconnected.add(client)
                        
                        # Wait for all sends to complete
                        if send_tasks:
                            results = await asyncio.gather(*send_tasks, return_exceptions=True)
                            # Track failed sends
                            for i, result in enumerate(results):
                                if isinstance(result, Exception):
                                    disconnected.add(list(clients_copy)[i])
                        
                        # Remove disconnected clients atomically
                        if disconnected:
                            async with self._lock:
                                self.clients -= disconnected
                        
                        # Log only on change or first reading
                        if distance != self._last_distance:
                            logger.info(f'Distance: {distance} cm')
                            self._last_distance = distance
                    else:
                        # Error case - pre-serialize once
                        error_data = {
                            'status': 'error',
                            'distance_cm': -1,
                            'distance_m': 0,
                            'distance_inch': 0,
                            'distance_feet': 0,
                            'error': 'Sensor timeout - check wiring',
                            'ts': timestamp
                        }
                        serialized_error = json.dumps(error_data)
                        
                        # Batch send errors
                        disconnected = set()
                        send_tasks = []
                        
                        for client in clients_copy:
                            try:
                                send_tasks.append(client.send(serialized_error))
                            except Exception:
                                disconnected.add(client)
                        
                        if send_tasks:
                            results = await asyncio.gather(*send_tasks, return_exceptions=True)
                            for i, result in enumerate(results):
                                if isinstance(result, Exception):
                                    disconnected.add(list(clients_copy)[i])
                        
                        if disconnected:
                            async with self._lock:
                                self.clients -= disconnected
                        
                        logger.warning('Sensor timeout - check wiring')
                        
                except Exception as e:
                    logger.error(f'Error reading sensor: {e}')
                    # Send error to clients
                    error_data = {
                        'status': 'error',
                        'distance_cm': -1,
                        'error': str(e),
                        'ts': int(time.time() * 1000)
                    }
                    serialized_error = json.dumps(error_data)
                    
                    async with self._lock:
                        clients_copy = self.clients.copy()
                    
                    disconnected = set()
                    for client in clients_copy:
                        try:
                            await client.send(serialized_error)
                        except Exception:
                            disconnected.add(client)
                    
                    if disconnected:
                        async with self._lock:
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
