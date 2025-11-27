// File: /Omega-Code/servers/robot-controller-backend/commands/init.go

// Package commands handles the processing and execution of various commands for the robot controller.
package commands

import (
    "runtime"

    "github.com/stianeikeland/go-rpio/v4"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/gpio"
)

// Global variables for GPIO interface and states
var (
    GpioInterface gpio.GPIO
    Low           rpio.State = 0
    High          rpio.State = 1
)

// InitGPIO initializes the GPIO interface based on the platform.
func InitGPIO() {
    if isRunningOnRaspberryPi() {
        GpioInterface = gpio.RealGPIO{}
        Low = rpio.Low
        High = rpio.High
    } else {
        GpioInterface = gpio.MockGPIO{}
    }
}

// isRunningOnRaspberryPi checks if the code is running on a Raspberry Pi.
func isRunningOnRaspberryPi() bool {
    return runtime.GOARCH == "arm" || runtime.GOARCH == "arm64"
}
