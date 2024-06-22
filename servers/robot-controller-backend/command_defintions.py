"""
Command Definitions

This script defines a set of commands for controlling various parts of the robot, such as motors, LEDs, servos, etc.

Class:
- COMMAND: Contains static command strings for different robot functions.
"""

class COMMAND:
    CMD_MOTOR = "CMD_MOTOR"
    CMD_LED = "CMD_LED"
    CMD_LED_MOD = "CMD_LED_MOD"
    CMD_SERVO_HORIZONTAL = "servo-horizontal"
    CMD_SERVO_VERTICAL = "servo-vertical"
    CMD_BUZZER = "buzz"
    CMD_BUZZER_STOP = "buzz-stop"
    CMD_SONIC = "CMD_SONIC"
    CMD_LIGHT = "CMD_LIGHT"
    CMD_POWER = "CMD_POWER"
    CMD_MODE = "CMD_MODE"
    MOVE_UP = "move-up"
    MOVE_DOWN = "move-down"
    MOVE_LEFT = "move-left"
    MOVE_RIGHT = "move-right"
    INCREASE_SPEED = "increase-speed"
    DECREASE_SPEED = "decrease-speed"

    def __init__(self):
        pass
