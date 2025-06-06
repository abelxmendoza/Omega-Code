"""
# File: /Omega-Code/servers/robot-controller-backend/command_definitions.py

Command Definitions

This script defines a set of commands for controlling various parts of the robot, such as motors, LEDs, servos, sensors, and more.

Class:
- COMMAND: Contains static command strings for different robot functions.
"""

class COMMAND:
    # Motor Commands
    CMD_MOTOR = "CMD_MOTOR"                  # Command to control the motors
    MOVE_UP = "move-up"                      # Command to move the robot up
    MOVE_DOWN = "move-down"                  # Command to move the robot down
    MOVE_LEFT = "move-left"                  # Command to move the robot left
    MOVE_RIGHT = "move-right"                # Command to move the robot right
    MOVE_STOP = "move-stop"                  # Command to stop specific directional movement

    # Speed Commands
    INCREASE_SPEED = "increase-speed"        # Command to increase the robot's speed
    DECREASE_SPEED = "decrease-speed"        # Command to decrease the robot's speed
    STOP = "stop"                            # Command to stop all movement

    # Servo Commands
    CMD_SERVO_HORIZONTAL = "servo-horizontal" # Command to control the horizontal servo
    CMD_SERVO_VERTICAL = "servo-vertical"     # Command to control the vertical servo
    SET_SERVO_POSITION = "set-servo-position" # Command to set specific servo angles
    RESET_SERVO = "reset-servo"               # Command to reset servos to their default position

    # LED Commands
    CMD_LED = "CMD_LED"                      # Command to control the LEDs
    CMD_LED_MOD = "CMD_LED_MOD"              # Command to control the LED mode
    SET_LED = "set-led"                      # Command to set the LED color and pattern
    LED_PATTERN = "led-pattern"              # Command to set LED patterns
    LED_TIMING = "led-timing"                # Command to adjust LED blinking/timing
    LED_BRIGHTNESS = "led-brightness"        # Command to control LED brightness

    # Lighting Control Commands
    LIGHTING_SET_COLOR = "lighting-set-color"   # Command to set LED color
    LIGHTING_SET_MODE = "lighting-set-mode"     # Command to set lighting mode (single, multi, two)
    LIGHTING_SET_PATTERN = "lighting-set-pattern" # Command to set lighting pattern (static, blink, fade)
    LIGHTING_SET_INTERVAL = "lighting-set-interval" # Command to set interval for dynamic patterns
    LIGHTING_TOGGLE = "lighting-toggle"         # Command to toggle lighting on/off

    # Buzzer Commands
    CMD_BUZZER = "buzz"                       # Command to activate the buzzer
    CMD_BUZZER_STOP = "buzz-stop"             # Command to stop the buzzer

    # Sensor Commands
    CMD_SONIC = "CMD_SONIC"                   # Command to control the ultrasonic sensor
    CMD_LIGHT = "CMD_LIGHT"                   # Command to control the light sensor

    # Power and Mode Commands
    CMD_POWER = "CMD_POWER"                   # Command to control the power
    CMD_MODE = "CMD_MODE"                     # Command to set the mode

    def __init__(self):
        pass
