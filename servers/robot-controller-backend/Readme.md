# Robot Controller Backend

This project is the backend server for the Omega-Code robot controller. Written in Go and Python, it handles incoming HTTP requests to control the robot's movements, speed, servos, and LED lights.

![1719168072062](image/Readme/1719168072062.png)

## Project Structure

The project is organized into several directories and files:

- **certs**: Contains SSL certificates for secure communication.
- **commands**: Command definitions and processors.
- **gpio**: GPIO handling code.
- **sensors**: Sensor interfacing code.
- **controllers**: Controllers for motors, servos, and line tracking.
- **video**: Video streaming server.
- **utils**: Utility scripts and helper classes.
- **tests**: Contains unit, integration, and end-to-end tests.
- **rust_module**: Rust integration for performance-critical tasks.
- **__pycache__**: Python bytecode cache.
- **venv**: Python virtual environment for dependencies.
- **main.go**: Entry point of the backend server (Go).
- **main_combined.go**: Starts the server along with the Python video server and ROS nodes.
- **server.csr**: Certificate Signing Request file.
- **server.log**: Log file for server activities.
- **go.mod**: Go module definitions.
- **go.sum**: Go module dependencies.

## Key Files

### PCA9685

- **PCA9685 Control**: `utils/PCA9685.py` - Controls the PCA9685 PWM driver.
- **Mock PCA9685**: `utils/mock_pca9685.py` - Mock implementation of PCA9685 for testing.

### LED Control

- **LED Control**: `utils/led_control.py` - Manages the LED lights on the robot.

### Servo Control

- **Servo Control**: `controllers/servo_control.py` - Controls the servo motors.

### ADC

- **ADC**: `sensors/adc.py` - Handles analog to digital conversion.

### Line Tracking

- **Line Tracking**: `controllers/line_tracking.py` - Manages the line tracking functionality.

### Threading Control

- **Threading Control**: `utils/threading_control.py` - Manages threading for concurrent tasks.

### Ultrasonic Sensor

- **Ultrasonic Sensor**: `sensors/ultrasonic_sensor.py` - Controls the ultrasonic sensor for distance measurement.
- **Ultrasonic Sensor (Go)**: `sensors/ultrasonic_sensor.go` - Handles ultrasonic sensor integration in Go.

### Command Definitions


 - **Command Definitions**: `command_definitions.py` - Defines commands for the robot.

- **Command Processor**: `commands/command_processor.py` - Processes incoming commands.

### Video Server

- **Video Server**: `video/video_server.py` - Handles video streaming from the robot.

## Getting Started

### Prerequisites

- Go
- Python

### Installation

1. Install Go dependencies:

   ```bash
   go get ./...
   ```
2. Set up Python virtual environment:

   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```
3. Set up environment variables:
   Create a `.env` file in the root directory with the following variables:

   ```env
   CERT_PATH=/path/to/cert.pem
   KEY_PATH=/path/to/key.pem
   ```

### Running the Project

1. Start the backend server with Go:

   ```bash
   go run main.go
   ```
   Or run the combined server:

   ```bash
   go run main_combined.go
   ```

### Usage

The backend server will listen for incoming HTTP requests to control the robot. Ensure the frontend is running to interact with the backend.

## Raspberry Pi 5 Compatibility

Omega-Code runs on the Raspberry Pi\&nbsp;5. The new board replaces the legacy `RPi.GPIO` interface used on the Pi\&nbsp;4\&nbsp;B with the `libgpiod` driver. Older scripts that import `RPi.GPIO` must be updated to use the `lgpio` package.

Install `lgpio` using pip:

```bash
pip install lgpio
```

Grant the GPIO group access to the device before running the software:

```bash
sudo chown root:gpio /dev/gpiochip0
sudo chmod g+rw /dev/gpiochip0
```

With these permissions in place, the rest of the project behaves just as it does on a Pi\&nbsp;4\&nbsp;B but benefits from the Pi\&nbsp;5's improved performance.

## Testing

This project includes a comprehensive test suite to ensure the functionality of the backend components.

### Test Structure

The tests are located in the `tests` directory with dedicated `unit`, `integration`, and `e2e` folders.

### Running Tests

To run the tests, follow these steps:

```bash
pip install -r requirements.txt
export PYTHONPATH=$(pwd)
pytest        # all tests
pytest tests/unit        # unit tests
pytest tests/integration # integration tests
pytest tests/e2e         # end-to-end tests
```

Run tests with `pytest`.


The tests are automatically run on each push to the master branch and on each pull request targeting the master branch using GitHub Actions. The CI/CD configuration can be found in the `.github/workflows/ci.yml` file.

### Test Descriptions
The suite covers modules such as LED control, servo control, and the mock PCA9685. See the `tests` directory for details.

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License.
