#!/bin/bash

# Determine project root. Allow override via OMEGA_CODE_ROOT
ROOT_DIR="${OMEGA_CODE_ROOT:-$(cd "$(dirname "$0")/../.." && pwd)}"

# Define the base directory for tests
BASE_DIR="$ROOT_DIR/servers/robot_controller_backend/tests"

# Create directories
mkdir -p $BASE_DIR/unit
mkdir -p $BASE_DIR/integration
mkdir -p $BASE_DIR/e2e
mkdir -p $BASE_DIR/utils

# Move existing test files to the appropriate directories
mv $BASE_DIR/conftest.py $BASE_DIR/utils/
mv $BASE_DIR/led_control_test.py $BASE_DIR/unit/
mv $BASE_DIR/main_test.go $BASE_DIR/unit/
mv $BASE_DIR/mock_pca9685_test.py $BASE_DIR/unit/
mv $BASE_DIR/servo_control_test.py $BASE_DIR/unit/

# Create __init__.py files for Python packages
touch $BASE_DIR/unit/__init__.py
touch $BASE_DIR/integration/__init__.py
touch $BASE_DIR/e2e/__init__.py
touch $BASE_DIR/utils/__init__.py

# Add example test files
echo '
import unittest

class TestCommandProcessor(unittest.TestCase):
    def test_example(self):
        self.assertEqual(1, 1)

if __name__ == "__main__":
    unittest.main()
' > $BASE_DIR/unit/test_command_processor.py

echo '
import unittest

class TestCommandIntegration(unittest.TestCase):
    def test_example(self):
        self.assertEqual(1, 1)

if __name__ == "__main__":
    unittest.main()
' > $BASE_DIR/integration/test_command_integration.py

echo '
import unittest

class TestFullSystem(unittest.TestCase):
    def test_example(self):
        self.assertEqual(1, 1)

if __name__ == "__main__":
    unittest.main()
' > $BASE_DIR/e2e/test_full_system.py

# Create conftest.py for shared fixtures and configurations
echo '
import pytest

@pytest.fixture(scope="session")
def setup_environment():
    # Setup code
    yield
    # Teardown code
' > $BASE_DIR/conftest.py

echo "Directory structure created and files organized successfully!"
