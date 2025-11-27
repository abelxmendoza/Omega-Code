"""
Optimized WebSocket message handler for improved performance.
- Message batching and compression
- Connection pooling
- Async message processing
- Memory optimization
"""

import asyncio
import json
import time
import zlib
from typing import Dict, List, Any, Optional, Callable
from collections import deque
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)

@dataclass
class MessageBatch:
    """Batch of messages for efficient processing"""
    messages: List[Dict[str, Any]]
    timestamp: float
    compressed: bool = False

class WebSocketOptimizer:
    """Optimized WebSocket message handler"""
    
    def __init__(self, batch_size: int = 10, batch_timeout: float = 0.05):
        self.batch_size = batch_size
        self.batch_timeout = batch_timeout
        self.message_queue = deque()
        self.last_batch_time = time.time()
        self.compression_threshold = 1024  # Compress messages larger than 1KB
        
    async def process_message(self, message: Dict[str, Any]) -> Optional[bytes]:
        """Process a single message with optimization"""
        # Add to batch
        self.message_queue.append(message)
        
        # Check if we should send batch
        if (len(self.message_queue) >= self.batch_size or 
            time.time() - self.last_batch_time >= self.batch_timeout):
            return await self._send_batch()
        
        return None
    
    async def _send_batch(self) -> bytes:
        """Send batched messages"""
        if not self.message_queue:
            return b""
        
        # Create batch
        batch = MessageBatch(
            messages=list(self.message_queue),
            timestamp=time.time()
        )
        
        # Clear queue
        self.message_queue.clear()
        self.last_batch_time = time.time()
        
        # Serialize batch
        batch_data = json.dumps({
            "type": "batch",
            "messages": batch.messages,
            "timestamp": batch.timestamp
        })
        
        # Compress if large enough
        if len(batch_data) > self.compression_threshold:
            compressed = zlib.compress(batch_data.encode())
            return compressed
        
        return batch_data.encode()

class ConnectionPool:
    """Connection pool for WebSocket optimization"""
    
    def __init__(self, max_connections: int = 100):
        self.max_connections = max_connections
        self.active_connections = {}
        self.connection_stats = {}
        
    async def get_connection(self, connection_id: str) -> Optional[Any]:
        """Get connection from pool"""
        if connection_id in self.active_connections:
            self.connection_stats[connection_id]["last_used"] = time.time()
            return self.active_connections[connection_id]
        return None
    
    async def add_connection(self, connection_id: str, connection: Any):
        """Add connection to pool"""
        if len(self.active_connections) >= self.max_connections:
            await self._cleanup_old_connections()
        
        self.active_connections[connection_id] = connection
        self.connection_stats[connection_id] = {
            "created": time.time(),
            "last_used": time.time(),
            "message_count": 0
        }
    
    async def _cleanup_old_connections(self):
        """Clean up old unused connections"""
        current_time = time.time()
        timeout = 300  # 5 minutes
        
        to_remove = []
        for conn_id, stats in self.connection_stats.items():
            if current_time - stats["last_used"] > timeout:
                to_remove.append(conn_id)
        
        for conn_id in to_remove:
            if conn_id in self.active_connections:
                del self.active_connections[conn_id]
            del self.connection_stats[conn_id]

class MessageProcessor:
    """Async message processor for WebSocket optimization"""
    
    def __init__(self, max_workers: int = 10):
        self.max_workers = max_workers
        self.worker_pool = asyncio.Queue(maxsize=max_workers)
        self.processors = []
        
    async def start(self):
        """Start message processors"""
        for i in range(self.max_workers):
            processor = asyncio.create_task(self._process_messages())
            self.processors.append(processor)
    
    async def stop(self):
        """Stop message processors"""
        for processor in self.processors:
            processor.cancel()
        await asyncio.gather(*self.processors, return_exceptions=True)
    
    async def _process_messages(self):
        """Process messages in worker thread"""
        while True:
            try:
                message = await self.worker_pool.get()
                await self._handle_message(message)
                self.worker_pool.task_done()
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Error processing message: {e}")
    
    async def _handle_message(self, message: Dict[str, Any]):
        """Handle individual message"""
        # Process message here
        pass
    
    async def queue_message(self, message: Dict[str, Any]):
        """Queue message for processing"""
        await self.worker_pool.put(message)
