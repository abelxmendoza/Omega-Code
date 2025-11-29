// File: /Omega-Code/servers/robot_controller_backend/gpio/init.go

package gpio

import "runtime"

// Global variable for GPIO interface
var GpioInterface GPIO

// InitGPIO initializes the GPIO interface based on the hardware type.
func InitGPIO() {
    if isRunningOnRaspberryPi() {
        GpioInterface = &RealGPIO{} // ✅ Fixed reference to RealGPIO
    } else {
        GpioInterface = &MockGPIO{} // ✅ Fixed reference to MockGPIO
    }
}

// isRunningOnRaspberryPi detects if the code is running on a Raspberry Pi.
func isRunningOnRaspberryPi() bool {
    return runtime.GOARCH == "arm" || runtime.GOARCH == "arm64"
}
