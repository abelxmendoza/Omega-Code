// File: /Omega-Code/ui/robot-controller-ui/src/control_definitions.ts

/*
This file defines a set of constant command strings used throughout the robot controller application.
These commands are used to control various aspects of the robot's functionality, such as movement, speed, and LED settings.
*/

export const COMMAND = {
    CMD_MOTOR: 'CMD_MOTOR',                   // Command to control the motors
    CMD_LED: 'CMD_LED',                       // Command to control the LEDs
    CMD_LED_MOD: 'CMD_LED_MOD',               // Command to control the LED mode
    CMD_SERVO_HORIZONTAL: 'servo-horizontal', // Command to control the horizontal servo
    CMD_SERVO_VERTICAL: 'servo-vertical',     // Command to control the vertical servo
    CMD_BUZZER: 'buzz',                       // Command to buzz the robot's horn
    CMD_BUZZER_STOP: 'buzz-stop',             // Command to stop the robot's horn
    CMD_SONIC: 'CMD_SONIC',                   // Command to control the sonic sensor
    CMD_LIGHT: 'CMD_LIGHT',                   // Command to control the light sensor
    CMD_POWER: 'CMD_POWER',                   // Command to control the power
    CMD_MODE: 'CMD_MODE',                     // Command to set the mode
    MOVE_UP: 'move-up',                       // Command to move the robot up
    MOVE_DOWN: 'move-down',                   // Command to move the robot down
    MOVE_LEFT: 'move-left',                   // Command to move the robot left
    MOVE_RIGHT: 'move-right',                 // Command to move the robot right
    INCREASE_SPEED: 'increase-speed',         // Command to increase the robot's speed
    DECREASE_SPEED: 'decrease-speed',         // Command to decrease the robot's speed
    SET_LED: 'set-led',                       // Command to set the LED color and pattern
    STOP: 'stop',                             // Command to stop all movement
    MOVE_STOP: 'move-stop',                   // Command to stop specific directional movement
};
