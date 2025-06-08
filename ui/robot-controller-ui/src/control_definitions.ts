/*
# File: /Omega-Code/ui/robot-controller-ui/src/control_definitions.ts
# Summary:
This file defines a centralized set of constant command strings and configurations for the robot controller application.
These commands are categorized by functionality, including motors, speed, servos, LEDs, sensors, and others.
It also includes constants for lighting modes and patterns for better control over LED behaviors.
*/

export const COMMAND = {
    // Motor Commands
    CMD_MOTOR: 'CMD_MOTOR',                   // General command for motor control
    MOVE_UP: 'move-up',                       // Move the robot forward
    MOVE_DOWN: 'move-down',                   // Move the robot backward
    MOVE_LEFT: 'move-left',                   // Move the robot to the left
    MOVE_RIGHT: 'move-right',                 // Move the robot to the right
    MOVE_STOP: 'move-stop',                   // Stop all specific directional movements

    // Speed Commands
    INCREASE_SPEED: 'increase-speed',         // Increase the robot's speed
    DECREASE_SPEED: 'decrease-speed',         // Decrease the robot's speed
    STOP: 'stop',                             // Emergency stop for all robot movement

    // Servo Commands
    CMD_SERVO_HORIZONTAL: 'servo-horizontal', // Adjust the horizontal servo position
    CMD_SERVO_VERTICAL: 'servo-vertical',     // Adjust the vertical servo position
    SET_SERVO_POSITION: 'set-servo-position', // Set specific angles for servos
    RESET_SERVO: 'reset-servo',               // Reset servos to their default positions

    // LED Commands
    CMD_LED: 'CMD_LED',                       // General command for controlling LEDs
    CMD_LED_MOD: 'CMD_LED_MOD',               // Modify LED mode settings
    SET_LED: 'set-led',                       // Set LED color and pattern
    LED_PATTERN: 'led-pattern',               // Define a specific LED pattern
    LED_TIMING: 'led-timing',                 // Adjust the timing for LED blinking
    LED_BRIGHTNESS: 'led-brightness',         // Control LED brightness levels

    // Lighting Control Commands
    LIGHTING_SET_COLOR: 'lighting-set-color',   // Define a specific LED color
    LIGHTING_SET_MODE: 'lighting-set-mode',     // Set the lighting mode (e.g., single, multi)
    LIGHTING_SET_PATTERN: 'lighting-set-pattern', // Specify a lighting pattern (e.g., static, fade)
    LIGHTING_SET_INTERVAL: 'lighting-set-interval', // Configure interval timing for dynamic patterns
    LIGHTING_TOGGLE: 'lighting-toggle',         // Turn lighting on or off

    // Buzzer Commands
    CMD_BUZZER: 'buzz',                       // Activate the robot's buzzer
    CMD_BUZZER_STOP: 'buzz-stop',             // Deactivate the buzzer

    // Sensor Commands
    CMD_SONIC: 'CMD_SONIC',                   // Manage ultrasonic sensor operations
    CMD_LIGHT: 'CMD_LIGHT',                   // Manage light sensor operations

    // Power and Mode Commands
    CMD_POWER: 'CMD_POWER',                   // Control robot power settings
    CMD_MODE: 'CMD_MODE',                     // Set operational mode for the robot
};

// Lighting Pattern Constants
export const LIGHTING_PATTERNS = [
    'static',   // LEDs maintain a constant color
    'blink',    // LEDs turn on and off at defined intervals
    'fade',     // LEDs transition smoothly between colors
    'chase',    // LEDs create a chasing light effect
    'rainbow'   // LEDs display a rainbow spectrum of colors
];

// Lighting Mode Constants
export const LIGHTING_MODES = [
    'single',   // All LEDs display the same color
    'multi',    // LEDs show multiple colors simultaneously
    'two'       // LEDs alternate between two colors
];
