// File: /Omega-Code/servers/robot-controller-backend/gpio/init.go

/*
Package gpio provides initialization functions for GPIO operations on both real and mock hardware.
It includes functions for setting up GPIO interfaces and detecting the hardware type.
*/

package gpio

import (
    "runtime"
    "github.com/stianeikeland/go-rpio/v4"
)

var (
    GpioInterface GPIO
    Low           rpio.State = 0
    High          rpio.State = 1
)

// InitGPIO initializes the GPIO interface based on the hardware type.
func InitGPIO() {
    if isRunningOnRaspberryPi() {
        GpioInterface = RealGPIO{}
        Low = rpio.Low
        High = rpio.High
    } else {
        GpioInterface = MockGPIO{}
    }
}

// isRunningOnRaspberryPi detects if the code is running on a Raspberry Pi.
func isRunningOnRaspberryPi() bool {
    return runtime.GOARCH == "arm" || runtime.GOARCH == "arm64"
}
