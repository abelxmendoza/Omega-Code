"""
Performance Stream Server - Real-time Beast Mode Data
- WebSocket server for real-time performance data
- Aggregates all beast mode system metrics
- Provides live dashboard data feed
- Coordinates with all optimization systems
"""

import asyncio
import websockets
import json
import logging
import time
from typing import Dict, Any, Set
from websockets.server import WebSocketServerProtocol

from .beast_mode_controller import beast_mode_controller
from .ai_predictive_maintenance import ai_predictive_maintenance
from .quantum_optimizer import quantum_optimizer
from .edge_computing import edge_optimizer
from .blockchain_verification import performance_verifier

logger = logging.getLogger(__name__)

class PerformanceStreamServer:
    """WebSocket server for real-time performance streaming"""
    
    def __init__(self, host: str = "localhost", port: int = 8088):
        self.host = host
        self.port = port
        self.connected_clients: Set[WebSocketServerProtocol] = set()
        self.running = False
        self.server = None
        
        # Data aggregation settings
        self.stream_interval = 1.0  # Send updates every second
        self.last_data = {}
    
    async def register_client(self, websocket: WebSocketServerProtocol):
        """Register a new WebSocket client"""
        try:
            self.connected_clients.add(websocket)
            logger.info(f"Performance stream client connected: {websocket.remote_address}")
            
            # Send initial data
            initial_data = await self.get_aggregated_performance_data()
            await websocket.send(json.dumps(initial_data))
            
        except Exception as e:
            logger.error(f"Client registration failed: {e}")
    
    async def unregister_client(self, websocket: WebSocketServerProtocol):
        """Unregister a WebSocket client"""
        try:
            self.connected_clients.discard(websocket)
            logger.info(f"Performance stream client disconnected: {websocket.remote_address}")
        except Exception as e:
            logger.error(f"Client unregistration failed: {e}")
    
    async def handle_client(self, websocket: WebSocketServerProtocol, path: str):
        """Handle WebSocket client connection"""
        try:
            await self.register_client(websocket)
            
            # Keep connection alive and handle messages
            async for message in websocket:
                try:
                    # Handle client messages (commands, requests, etc.)
                    await self.handle_client_message(websocket, message)
                except Exception as e:
                    logger.error(f"Message handling failed: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info("Client connection closed")
        except Exception as e:
            logger.error(f"Client handling failed: {e}")
        finally:
            await self.unregister_client(websocket)
    
    async def handle_client_message(self, websocket: WebSocketServerProtocol, message: str):
        """Handle incoming client messages"""
        try:
            data = json.loads(message)
            command = data.get("command")
            
            if command == "get_status":
                response = await self.get_aggregated_performance_data()
                await websocket.send(json.dumps(response))
            elif command == "start_beast_mode":
                beast_mode_controller.start_beast_mode()
                response = {"status": "beast_mode_started"}
                await websocket.send(json.dumps(response))
            elif command == "stop_beast_mode":
                beast_mode_controller.stop_beast_mode()
                response = {"status": "beast_mode_stopped"}
                await websocket.send(json.dumps(response))
            else:
                response = {"error": f"Unknown command: {command}"}
                await websocket.send(json.dumps(response))
                
        except json.JSONDecodeError:
            logger.error("Invalid JSON received from client")
        except Exception as e:
            logger.error(f"Client message handling failed: {e}")
    
    async def get_aggregated_performance_data(self) -> Dict[str, Any]:
        """Get aggregated performance data from all systems"""
        try:
            # Get beast mode controller status
            beast_mode_status = beast_mode_controller.get_beast_mode_status()
            
            # Get individual system stats
            ai_stats = ai_predictive_maintenance.get_ai_performance_stats()
            quantum_stats = quantum_optimizer.get_quantum_performance_stats()
            edge_stats = edge_optimizer.get_edge_performance_stats()
            blockchain_stats = performance_verifier.get_verification_stats()
            
            # Aggregate all data
            aggregated_data = {
                "timestamp": time.time(),
                "beast_mode": beast_mode_status,
                "ai_predictive": ai_stats,
                "quantum_optimizer": quantum_stats,
                "edge_computing": edge_stats,
                "blockchain_verification": blockchain_stats,
                "system_metrics": {
                    "cpu_usage": beast_mode_status.get("performance_snapshot", {}).get("subsystem_scores", {}).get("hardware_performance", 0),
                    "memory_usage": 0,  # Would get from system monitoring
                    "network_latency": 0,
                    "response_time": 0,
                    "error_rate": 0
                }
            }
            
            return aggregated_data
            
        except Exception as e:
            logger.error(f"Performance data aggregation failed: {e}")
            return {"error": str(e), "timestamp": time.time()}
    
    async def broadcast_performance_data(self):
        """Broadcast performance data to all connected clients"""
        try:
            if not self.connected_clients:
                return
            
            data = await self.get_aggregated_performance_data()
            
            # Only send if data has changed significantly
            if self._should_send_update(data):
                message = json.dumps(data)
                
                # Send to all connected clients
                disconnected_clients = set()
                for client in self.connected_clients:
                    try:
                        await client.send(message)
                    except websockets.exceptions.ConnectionClosed:
                        disconnected_clients.add(client)
                    except Exception as e:
                        logger.error(f"Failed to send data to client: {e}")
                        disconnected_clients.add(client)
                
                # Remove disconnected clients
                self.connected_clients -= disconnected_clients
                
                # Update last data
                self.last_data = data
                
        except Exception as e:
            logger.error(f"Performance data broadcast failed: {e}")
    
    def _should_send_update(self, new_data: Dict[str, Any]) -> bool:
        """Check if update should be sent based on data changes"""
        try:
            if not self.last_data:
                return True
            
            # Check if overall score changed significantly
            old_score = self.last_data.get("beast_mode", {}).get("performance_snapshot", {}).get("overall_score", 0)
            new_score = new_data.get("beast_mode", {}).get("performance_snapshot", {}).get("overall_score", 0)
            
            if abs(new_score - old_score) > 0.5:  # 0.5 point change
                return True
            
            # Check if beast mode level changed
            old_level = self.last_data.get("beast_mode", {}).get("current_level", "")
            new_level = new_data.get("beast_mode", {}).get("current_level", "")
            
            if old_level != new_level:
                return True
            
            # Send update every 5 seconds regardless
            time_diff = new_data.get("timestamp", 0) - self.last_data.get("timestamp", 0)
            if time_diff > 5.0:
                return True
            
            return False
            
        except Exception as e:
            logger.error(f"Update check failed: {e}")
            return True
    
    async def start_streaming(self):
        """Start the performance streaming loop"""
        try:
            while self.running:
                await self.broadcast_performance_data()
                await asyncio.sleep(self.stream_interval)
                
        except Exception as e:
            logger.error(f"Streaming loop failed: {e}")
    
    async def start_server(self):
        """Start the WebSocket server"""
        try:
            self.running = True
            
            # Start WebSocket server
            self.server = await websockets.serve(
                self.handle_client,
                self.host,
                self.port,
                ping_interval=20,
                ping_timeout=10
            )
            
            logger.info(f"Performance stream server started on {self.host}:{self.port}")
            
            # Start streaming task
            streaming_task = asyncio.create_task(self.start_streaming())
            
            # Wait for server to run
            await self.server.wait_closed()
            
        except Exception as e:
            logger.error(f"Performance stream server failed: {e}")
    
    async def stop_server(self):
        """Stop the WebSocket server"""
        try:
            self.running = False
            
            if self.server:
                self.server.close()
                await self.server.wait_closed()
            
            # Close all client connections
            for client in self.connected_clients.copy():
                await client.close()
            
            logger.info("Performance stream server stopped")
            
        except Exception as e:
            logger.error(f"Server shutdown failed: {e}")

# Global performance stream server
performance_stream_server = PerformanceStreamServer()

# Async function to run the server
async def run_performance_stream_server():
    """Run the performance stream server"""
    try:
        await performance_stream_server.start_server()
    except KeyboardInterrupt:
        await performance_stream_server.stop_server()
    except Exception as e:
        logger.error(f"Performance stream server error: {e}")

if __name__ == "__main__":
    # Run the server
    asyncio.run(run_performance_stream_server())

