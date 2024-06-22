# Omega-Code

Omega-Code is a comprehensive project aimed at developing a fully-featured robot controller interface. The project integrates a backend server for handling robot commands and a frontend user interface for controlling the robot's movements, camera, and LED lights. The project is structured to facilitate ease of use and maintainability, with clear separation of concerns between the backend and frontend components.

## Project Structure

The project is organized into several directories:

- **config**: Contains configuration files necessary for the project.
- **scripts**: Contains shell scripts for connecting to a hotspot.
- **servers**: Contains the backend server code for handling robot commands.
- **ui**: Contains the frontend user interface code for the robot controller.

## Backend Server

The backend server, written in Go, handles incoming HTTP requests to control the robot. It includes command execution for movements, speed control, servo control, and LED control.

### Key Files

- **main.go**: The entry point of the backend server. It sets up the server, loads environment variables, and handles incoming commands.
- **servo_control.py**: A Python script used by the backend server to control servo motors.

## Frontend User Interface

The frontend user interface, built with Next.js and React, provides a web-based interface for controlling the robot. It includes features such as a video feed, control panels, speed control, command log, and LED control.

### Key Files and Components

- **src/pages/index.tsx**: The main page of the application. It integrates all the components and sets up keyboard controls for the robot.
- **src/components/CommandLogContext.tsx**: Provides a context for logging commands sent to the robot.
- **src/components/ControlPanel.tsx**: A component for controlling the robot's movements.
- **src/components/SpeedControl.tsx**: A component for controlling the robot's speed.
- **src/components/LedControl.tsx**: A component for controlling the robot's LED lights.
- **src/components/VideoFeed.tsx**: A component for displaying the robot's video feed.
- **src/components/Header.tsx**: The header component showing the robot's connection status and battery level.
- **src/components/Status.tsx**: Displays the robot's connection status and battery level.
- **src/components/LedModal.tsx**: A modal component for configuring LED settings.
- **src/components/LightingPattern.tsx**: A component for selecting lighting patterns.
- **src/components/LightingMode.tsx**: A component for selecting lighting modes.
- **src/components/IntervalTiming.tsx**: A component for setting the interval timing for LED patterns.
- **src/components/ControlButtons.tsx**: A component for start, stop, and apply settings buttons.
- **src/components/ColorWheel.tsx**: A component for selecting colors using a color wheel.

### Command Definitions

The `control_definitions.ts` file contains command definitions used throughout the project.

## Getting Started

### Prerequisites

- Node.js
- Go
- Python

### Installation

1. **Install backend dependencies:**

   ```sh
   cd servers/robot-controller-backend
   go get ./...
   ```
2. **Install frontend dependencies:**

   ```sh
   cd ui/robot-controller-ui
   npm install
   ```
3. **Set up environment variables:**

   Create a `.env` file in the `servers/robot-controller-backend` directory with the following variables:

   ```sh
   CERT_PATH=/path/to/cert.pem
   KEY_PATH=/path/to/key.pem
   ```

### Running the Project

1. **Start the backend server:**

   ```sh
   cd servers/robot-controller-backend
   go run main.go
   ```
2. **Start the frontend development server:**

   ```sh
   cd ui/robot-controller-ui
   npm run dev
   ```

### Usage

Open a web browser and navigate to https://localhost:3000 to access the robot controller interface. Use the provided controls to send commands to the robot.

### Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any improvements or bug fixes.

### License

This project is licensed under the MIT License.

### Acknowledgements

Special thanks to all contributors and the open-source community for their support and contributions.
