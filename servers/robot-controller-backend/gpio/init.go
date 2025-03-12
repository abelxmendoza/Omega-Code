/*
# File: /Omega-Code/servers/robot-controller-backend/gpio/init.go
# Summary:
Initializes GPIO interface for Raspberry Pi or a mock version for testing.
*/

package gpio

import (
	"fmt"
	"log"
	"runtime"

	"github.com/stianeikeland/go-rpio/v4"
)

// InitGPIO initializes the GPIO interface based on the hardware type.
func InitGPIO() {
	if isRunningOnRaspberryPi() {
		log.Println("🟢 Running on Raspberry Pi - Using Real GPIO")
		if err := rpio.Open(); err != nil {
			log.Fatalf("❌ Failed to initialize GPIO: %v", err)
		}
		GpioInterface = RealGPIO{}
	} else {
		log.Println("⚠️ Running on non-Raspberry Pi system - Using Mock GPIO")
		GpioInterface = MockGPIO{}
	}
}

// CleanupGPIO ensures the GPIO pins are properly closed before shutting down.
func CleanupGPIO() {
	if isRunningOnRaspberryPi() {
		log.Println("🔴 Cleaning up GPIO...")
		if err := rpio.Close(); err != nil {
			log.Printf("⚠️ Error closing GPIO: %v", err)
		}
	}
}

// isRunningOnRaspberryPi detects if the code is running on a Raspberry Pi.
func isRunningOnRaspberryPi() bool {
	return runtime.GOARCH == "arm" || runtime.GOARCH == "arm64"
}
