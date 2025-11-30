#!/usr/bin/env bash
# Start lighting server and keep it running
# This script ensures the lighting server is running on port 8082

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if already running
if pgrep -f "main_lighting.go" > /dev/null; then
    PID=$(pgrep -f "main_lighting.go")
    echo "⚠️  Lighting server already running (PID: $PID)"
    echo "   To restart, stop it first: pkill -f main_lighting.go"
    exit 0
fi

# Check if port is in use
if sudo lsof -i :8082 >/dev/null 2>&1 || sudo ss -tlnp | grep -q ":8082"; then
    echo "❌ Port 8082 is already in use"
    echo "   Check what's using it: sudo lsof -i :8082"
    exit 1
fi

echo "🚀 Starting lighting server..."
echo "   Working directory: $(pwd)"
echo "   Port: 8082"
echo "   Path: /lighting"
echo ""

# Set environment variables
export PORT_LIGHTING=8082
export LIGHTING_PATH=/lighting

# Start server in background
nohup go run main_lighting.go > lighting.log 2>&1 &
SERVER_PID=$!

# Wait a moment for startup
sleep 2

# Check if process is still running
if ! kill -0 $SERVER_PID 2>/dev/null; then
    echo "❌ Server failed to start. Check lighting.log for errors:"
    tail -20 lighting.log
    exit 1
fi

# Test health endpoint
if curl -s --max-time 2 http://127.0.0.1:8082/health >/dev/null 2>&1; then
    echo "✅ Server started successfully (PID: $SERVER_PID)"
    echo "   Health check: http://127.0.0.1:8082/health"
    echo "   Tailscale: http://100.93.225.61:8082/health"
    echo "   Logs: tail -f lighting.log"
else
    echo "⚠️  Server started but health check failed"
    echo "   PID: $SERVER_PID"
    echo "   Check logs: tail -f lighting.log"
fi

echo ""
echo "💡 To stop: pkill -f main_lighting.go"
echo "💡 To view logs: tail -f lighting.log"

