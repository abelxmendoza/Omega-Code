#!/bin/bash
# Standalone Server Runner
# Run individual servers independently without the main backend

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_DIR="$(cd "$SCRIPT_DIR/../servers/robot_controller_backend" && pwd)"
cd "$BACKEND_DIR"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_usage() {
    echo "Usage: $0 [server]"
    echo ""
    echo "Available servers:"
    echo "  ultrasonic  - Ultrasonic sensor (port 8080)"
    echo "  movement   - Movement control (port 8081)"
    echo "  lighting   - LED lighting (port 8082)"
    echo "  line       - Line tracker (port 8090)"
    echo "  video      - Video streaming (port 5000)"
    echo ""
    echo "Examples:"
    echo "  $0 ultrasonic    # Run ultrasonic server only"
    echo "  $0 movement      # Run movement server only"
    echo ""
    echo "Environment variables can be set before running:"
    echo "  PORT_ULTRASONIC=8080"
    echo "  PORT_MOVEMENT=8081"
    echo "  ORIGIN_ALLOW=http://localhost:3000"
}

run_ultrasonic() {
    echo -e "${GREEN}Starting Ultrasonic WebSocket Server (port 8080)...${NC}"
    cd "$BACKEND_DIR/sensors"
    python3 ultrasonic_ws_server.py
}

run_movement() {
    echo -e "${GREEN}Starting Movement WebSocket Server (port 8081)...${NC}"
    cd "$BACKEND_DIR/movement"
    python3 movement_ws_server.py
}

run_lighting() {
    echo -e "${GREEN}Lighting is handled by the main FastAPI backend.${NC}"
    echo -e "${YELLOW}Start the full backend with: python main_api.py${NC}"
    exit 1
}

run_line() {
    echo -e "${GREEN}Starting Line Tracker WebSocket Server (port 8090)...${NC}"
    cd "$BACKEND_DIR/sensors"
    python3 line_tracking_ws_server.py
}

run_video() {
    echo -e "${GREEN}Starting Video Server (port 5000)...${NC}"
    cd "$BACKEND_DIR/video"
    python3 video_server.py
}

# Main
if [ $# -eq 0 ]; then
    print_usage
    exit 1
fi

case "$1" in
    ultrasonic)
        run_ultrasonic
        ;;
    movement)
        run_movement
        ;;
    lighting)
        run_lighting
        ;;
    line|line-tracker)
        run_line
        ;;
    video)
        run_video
        ;;
    *)
        echo -e "${YELLOW}Unknown server: $1${NC}"
        print_usage
        exit 1
        ;;
esac

