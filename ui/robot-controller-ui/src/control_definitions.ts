// File: /Omega-Code/ui/robot-controller-ui/src/control_definitions.ts

/*
This file defines a set of constant command strings used throughout the robot controller application.
These commands are used to control various aspects of the robot's functionality, such as movement, speed, and LED settings.
*/

export const COMMAND = {
    CMD_SERVO_HORIZONTAL: 'servo-horizontal', // Command to control the horizontal servo
    CMD_SERVO_VERTICAL: 'servo-vertical',     // Command to control the vertical servo
    MOVE_UP: 'move-up',                       // Command to move the robot up
    MOVE_DOWN: 'move-down',                   // Command to move the robot down
    MOVE_LEFT: 'move-left',                   // Command to move the robot left
    MOVE_RIGHT: 'move-right',                 // Command to move the robot right
    INCREASE_SPEED: 'increase-speed',         // Command to increase the robot's speed
    DECREASE_SPEED: 'decrease-speed',         // Command to decrease the robot's speed
    HONK: 'honk',                             // Command to honk the robot's horn
    SET_LED: 'set-led'                        // Command to set the LED color and pattern
};
