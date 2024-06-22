# Omega-Code

Omega-Code is a comprehensive project aimed at developing a fully-featured robot controller interface. The project integrates a backend server for handling robot commands and a frontend user interface for controlling the robot's movements, camera, and LED lights. The project is structured to facilitate ease of use and maintainability, with clear separation of concerns between the backend and frontend components.

<img src="image/README/1719064424761.png" alt="1719064424761" width="400"/>

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

The frontend UI, built with Next.js and React, provides a web-based interface for controlling the robot. Features include a video feed, control panels, speed control, command log, and LED control.

### Key Files and Components

- `src/pages/index.tsx`: Main page integrating all components.
- `src/components/CommandLogContext.tsx`: Context for logging commands.
- `src/components/ControlPanel.tsx`: Component for controlling robot movements.
- `src/components/SpeedControl.tsx`: Component for controlling robot speed.
- `src/components/LedControl.tsx`: Component for controlling LED lights.
- `src/components/VideoFeed.tsx`: Component for displaying video feed.
- `src/components/Header.tsx`: Displays connection status and battery level.
- `src/components/Status.tsx`: Displays connection status and battery level.
- `src/components/LedModal.tsx`: Modal for configuring LED settings.
- `src/components/LightingPattern.tsx`: Selects lighting patterns.
- `src/components/LightingMode.tsx`: Selects lighting modes.
- `src/components/IntervalTiming.tsx`: Sets interval timing for LED patterns.
- `src/components/ControlButtons.tsx`: Start, stop, and apply settings buttons.
- `src/components/ColorWheel.tsx`: Color selection using a color wheel.
- `control_definitions.ts`: Command definitions used throughout the project.

## Getting Started

### Prerequisites

- Node.js
- Go
- Python

### Installation

1. Install backend dependencies:
   ```bash
   cd servers/robot-controller-backend
   go get ./...
   ```
2. Install frontend dependencies:
   ```bash
   cd ui/robot-controller-ui
   npm install
   ```
3. Set up environment variables:
   Create a `.env` file in the `servers/robot-controller-backend` directory with the following variables:
   ```env
   CERT_PATH=/path/to/cert.pem
   KEY_PATH=/path/to/key.pem
   ```

### Running the Project

1. Start the backend server:
   ```bash
   cd servers/robot-controller-backend
   go run main.go
   ```
2. Start the frontend development server:
   ```bash
   cd ui/robot-controller-ui
   npm run dev
   ```

### Usage

Open a web browser and navigate to `https://localhost:3000` to access the robot controller interface. Use the provided controls to send commands to the robot.

## Acknowledgements

Special thanks to Freenove for providing the **Freenove 4WD Smart Car Kit for Raspberry Pi** and their comprehensive support.

### Freenove 4WD Smart Car Kit for Raspberry Pi

> A 4WD smart car kit for Raspberry Pi.

![Freenove](Picture/icon.png)
![Freenove](Picture/icon1.png)

#### Download

- **Use command in console:**
  ```bash
  git clone https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi.git
  ```
- **Manually download in browser:**
  Click the green "Clone or download" button, then click "Download ZIP" button in the pop-up window.

For any difficulties, please contact Freenove support at [support@freenove.com](mailto:support@freenove.com).

#### Support

Freenove provides free and quick customer support, including but not limited to:

- Quality problems of products
- Using problems of products
- Questions of learning and creation
- Opinions and suggestions
- Ideas and thoughts

#### Purchase

Visit [Freenove Store](http://store.freenove.com) to purchase their products. For business inquiries, contact [sale@freenove.com](mailto:sale@freenove.com).

#### Copyright

All files in the Freenove repository are released under [Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License](http://creativecommons.org/licenses/by-nc-sa/3.0/).

![Creative Commons](https://i.creativecommons.org/l/by-nc-sa/3.0/88x31.png)

Freenove brand and logo are copyright of Freenove Creative Technology Co., Ltd. Can't be used without formal permission.

#### About Freenove

Freenove is an open-source electronics platform committed to helping customers quickly realize creative ideas and product prototypes. They offer robot kits, learning kits for Arduino, Raspberry Pi, and micro:bit, electronic components and modules, tools, and product customization services.

## License

This project is licensed under the MIT License.
