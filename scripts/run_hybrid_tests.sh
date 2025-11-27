#!/bin/bash
# Run Hybrid System Tests with Mock Orin

set -e

echo "🧪 Running Hybrid System Tests with Mock Orin..."

cd servers/robot-controller-backend

# Activate virtual environment if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Install test dependencies
pip install -q -r requirements-test.txt

# Run hybrid tests
echo "Running hybrid mode switching tests..."
pytest tests/hybrid/test_hybrid_mode_switching.py -v

echo "Running mode WebSocket update tests..."
pytest tests/hybrid/test_mode_ws_updates.py -v

echo "Running thermal to mode transition tests..."
pytest tests/hybrid/test_thermal_to_mode_transition.py -v

# Run fault injection tests
echo "Running fault injection tests..."
pytest tests/faults/ -v

echo "✅ Hybrid system tests completed!"

