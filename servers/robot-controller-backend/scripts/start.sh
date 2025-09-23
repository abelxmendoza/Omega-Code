#!/bin/bash

# Robot Controller Backend Startup Script
# This script handles the startup sequence for all backend services

set -e

echo "🤖 Starting Robot Controller Backend..."

# Function to check if a port is available
check_port() {
    local port=$1
    local service=$2
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null ; then
        echo "⚠️  Port $port is already in use by $service"
        return 1
    else
        echo "✅ Port $port is available"
        return 0
    fi
}

# Function to wait for a service to be ready
wait_for_service() {
    local host=$1
    local port=$2
    local service=$3
    local max_attempts=30
    local attempt=1
    
    echo "⏳ Waiting for $service to be ready..."
    
    while [ $attempt -le $max_attempts ]; do
        if nc -z $host $port 2>/dev/null; then
            echo "✅ $service is ready!"
            return 0
        fi
        
        echo "Attempt $attempt/$max_attempts: $service not ready yet..."
        sleep 2
        attempt=$((attempt + 1))
    done
    
    echo "❌ $service failed to start after $max_attempts attempts"
    return 1
}

# Check required ports
echo "🔍 Checking port availability..."
check_port 8080 "Main API" || exit 1
check_port 8081 "Movement WebSocket" || exit 1
check_port 8090 "Sensor WebSocket" || exit 1

# Set environment variables
export PYTHONPATH="${PYTHONPATH}:$(pwd)"
export ENVIRONMENT="${ENVIRONMENT:-production}"
export LOG_LEVEL="${LOG_LEVEL:-INFO}"

# Create logs directory
mkdir -p logs

# Function to start a service in background
start_service() {
    local service_name=$1
    local command=$2
    local log_file="logs/${service_name}.log"
    
    echo "🚀 Starting $service_name..."
    nohup $command > $log_file 2>&1 &
    local pid=$!
    echo $pid > "logs/${service_name}.pid"
    echo "✅ $service_name started with PID $pid"
}

# Start services in order
echo "📡 Starting WebSocket servers..."

# Start Movement WebSocket Server
start_service "movement_ws_server" "python movement/movement_ws_server.py"

# Start Line Tracking WebSocket Server
start_service "line_tracking_ws_server" "python sensors/line_tracking_ws_server.py"

# Start Ultrasonic WebSocket Server (if Go service exists)
if [ -f "sensors/main_ultrasonic.go" ]; then
    start_service "ultrasonic_ws_server" "go run sensors/main_ultrasonic.go"
fi

# Wait for WebSocket servers to be ready
wait_for_service "localhost" 8081 "Movement WebSocket Server" || exit 1
wait_for_service "localhost" 8090 "Line Tracking WebSocket Server" || exit 1

# Start Main API Server
echo "🌐 Starting Main API Server..."
start_service "main_api" "python main_api.py"

# Wait for API server to be ready
wait_for_service "localhost" 8080 "Main API Server" || exit 1

# Health check
echo "🏥 Performing health checks..."
sleep 5

# Check if all services are responding
services=("8080:Main API" "8081:Movement WebSocket" "8090:Sensor WebSocket")
all_healthy=true

for service in "${services[@]}"; do
    port=$(echo $service | cut -d: -f1)
    name=$(echo $service | cut -d: -f2)
    
    if curl -f -s "http://localhost:$port/health" > /dev/null 2>&1; then
        echo "✅ $name health check passed"
    else
        echo "❌ $name health check failed"
        all_healthy=false
    fi
done

if [ "$all_healthy" = true ]; then
    echo "🎉 All services are healthy and ready!"
    echo "📊 Service Status:"
    echo "   Main API: http://localhost:8080"
    echo "   Movement WebSocket: ws://localhost:8081/"
    echo "   Sensor WebSocket: ws://localhost:8090/"
    echo "   Logs: ./logs/"
else
    echo "❌ Some services failed health checks"
    exit 1
fi

# Keep the script running and monitor services
echo "👀 Monitoring services..."
while true; do
    sleep 30
    
    # Check if all PIDs are still running
    for service in movement_ws_server line_tracking_ws_server ultrasonic_ws_server main_api; do
        if [ -f "logs/${service}.pid" ]; then
            pid=$(cat "logs/${service}.pid")
            if ! kill -0 $pid 2>/dev/null; then
                echo "❌ $service (PID $pid) has stopped unexpectedly"
                # Restart the service
                echo "🔄 Restarting $service..."
                case $service in
                    "movement_ws_server")
                        start_service "movement_ws_server" "python movement/movement_ws_server.py"
                        ;;
                    "line_tracking_ws_server")
                        start_service "line_tracking_ws_server" "python sensors/line_tracking_ws_server.py"
                        ;;
                    "ultrasonic_ws_server")
                        start_service "ultrasonic_ws_server" "go run sensors/main_ultrasonic.go"
                        ;;
                    "main_api")
                        start_service "main_api" "python main_api.py"
                        ;;
                esac
            fi
        fi
    done
done
