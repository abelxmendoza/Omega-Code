"""
WebSocket Server for Video Metrics
Provides real-time metrics and status updates via WebSocket.
"""

import json
import time
import threading
import logging
from typing import Set, Dict, Any, Optional
from queue import Queue, Empty

try:
    from flask import request
    from flask_socketio import SocketIO, emit, disconnect
    SOCKETIO_AVAILABLE = True
except ImportError:
    SOCKETIO_AVAILABLE = False

try:
    import websockets
    import asyncio
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False

log = logging.getLogger(__name__)


class VideoMetricsWebSocket:
    """
    WebSocket server for real-time video metrics.
    Supports both Flask-SocketIO and native websockets.
    """
    
    def __init__(self, app=None):
        """Initialize WebSocket server."""
        self.app = app
        self.clients: Set[str] = set()
        self._metrics_queue: Queue = Queue()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self.socketio = None
        
        if SOCKETIO_AVAILABLE and app:
            try:
                self.socketio = SocketIO(app, cors_allowed_origins="*")
                self._setup_socketio()
            except Exception as e:
                log.warning(f"Failed to initialize Flask-SocketIO: {e}")
                self.socketio = None
        elif WEBSOCKETS_AVAILABLE:
            self._setup_native_websockets()
        else:
            log.debug("No WebSocket library available (flask-socketio or websockets)")
    
    def _setup_socketio(self):
        """Setup Flask-SocketIO handlers."""
        if not self.socketio:
            return
        
        @self.socketio.on('connect')
        def handle_connect():
            client_id = request.sid
            self.clients.add(client_id)
            log.info(f"WebSocket client connected: {client_id}")
            emit('connected', {'status': 'ok', 'client_id': client_id})
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            client_id = request.sid
            self.clients.discard(client_id)
            log.info(f"WebSocket client disconnected: {client_id}")
        
        @self.socketio.on('subscribe_metrics')
        def handle_subscribe():
            client_id = request.sid
            log.info(f"Client {client_id} subscribed to metrics")
            emit('subscribed', {'status': 'ok'})
    
    def _setup_native_websockets(self):
        """Setup native websockets server (fallback)."""
        # Would need async server setup - can be implemented if needed
        pass
    
    def broadcast_metrics(self, metrics: Dict[str, Any]):
        """Broadcast metrics to all connected clients."""
        if not self.socketio:
            return
        
        try:
            self.socketio.emit('metrics', metrics, broadcast=True)
        except Exception as e:
            log.warning(f"Failed to broadcast metrics: {e}")
    
    def start(self):
        """Start WebSocket server."""
        self._running = True
        if self.socketio:
            log.info("WebSocket server started (Flask-SocketIO)")
    
    def stop(self):
        """Stop WebSocket server."""
        self._running = False
        self.clients.clear()

