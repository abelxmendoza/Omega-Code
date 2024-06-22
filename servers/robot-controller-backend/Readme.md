# Robot Controller Backend

This project is the backend server for the Omega-Code robot controller. Written in Go and Python, it handles incoming HTTP requests to control the robot's movements, speed, servos, and LED lights.

## Project Structure

The project is organized into several directories and files:

- **certs**: Contains SSL certificates for secure communication.
- **__pycache__**: Python bytecode cache.
- **venv**: Python virtual environment for dependencies.
- **main.go**: Entry point of the backend server (Go).
- **main.py**: Entry point of the backend server (Python).
- **server.csr**: Certificate Signing Request file.
- **server.log**: Log file for server activities.
- **go.mod**: Go module definitions.
- **go.sum**: Go module dependencies.

### Key Files

### PCA9685

- **PCA9685 Control**: `PCA9685.py` - Controls the PCA9685 PWM driver.

### LED Control

- **LED Control**: `led_control.py` - Manages the LED lights on the robot.

### Servo Control

- **Servo Control**: `servo_control.py` - Controls the servo motors.

### ADC

- **ADC**: `adc.py` - Handles analog to digital conversion.

### Line Tracking

- **Line Tracking**: `line_tracking.py` - Manages the line tracking functionality.

### Threading Control

- **Threading Control**: `threading_control.py` - Manages threading for concurrent tasks.

### Ultrasonic Sensor

- **Ultrasonic Sensor**: `ultrasonic_sensor.py` - Controls the ultrasonic sensor for distance measurement.

### Command Definitions

- **Command Definitions**: `command_defintions.py` - Defines commands for the robot.

### Mock PCA9685

- **Mock PCA9685**: `mock_pca9685.py` - Mock implementation of PCA9685 for testing.

### Video Server

- **Video Server**: `video_server.py` - Handles video streaming from the robot.

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

1. Start the backend server (Go):

   ```bash
   go run main.go
   ```
2. Start the backend server (Python):

   ```bash
   python main.py
   ```

### Usage

The backend server will listen for incoming HTTP requests to control the robot. Ensure the frontend is running to interact with the backend.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License.
