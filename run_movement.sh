#!/bin/bash
# Run Movement WebSocket Server
# Usage: ./run_movement.sh

cd "$(dirname "$0")"
python3 -m servers.robot_controller_backend.movement.movement_ws_server "$@"

