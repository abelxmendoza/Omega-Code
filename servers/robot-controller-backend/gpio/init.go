/*
# File: /Omega-Code/servers/robot-controller-backend/gpio/init.go
# Summary:
Initializes GPIO interface for Raspberry Pi or a mock version for testing.
*/

package gpio

import (
	"runtime"

	"github.com/stianeikeland/go-rpio/v4"
)

// Global variables for GPIO interface and states
var (
	GpioInterface GPIO
	Low           rpio.State = rpio.Low
	High          rpio.State = rpio.High
)

// InitGPIO initializes the GPIO interface based on the hardware type.
func InitGPIO() {
	if isRunningOnRaspberryPi() {
		GpioInterface = RealGPIO{}
	} else {
		GpioInterface = MockGPIO{}
	}
}

// isRunningOnRaspberryPi detects if the code is running on a Raspberry Pi.
func isRunningOnRaspberryPi() bool {
	return runtime.GOARCH == "arm" || runtime.GOARCH == "arm64"
}

