/*
# File: /Omega-Code/servers/robot-controller-backend/gpio/motor.go
# Summary:
Controls the car's motors for forward and backward movement.
*/

package gpio

import (
	"log"
	"github.com/stianeikeland/go-rpio/v4"
)

var (
	// Define GPIO pins for forward and backward movement
	forwardPin  = rpio.Pin(19)
	backwardPin = rpio.Pin(20)
)

// InitMotor initializes GPIO pins for motor control.
func InitMotor() error {
	err := rpio.Open()
	if err != nil {
		return err
	}
	forwardPin.Output()
	backwardPin.Output()
	return nil
}

// ActivateMotor moves the car forward or backward.
func ActivateMotor(direction string) {
	if direction == "forward" {
		forwardPin.High()
		backwardPin.Low()
		log.Println("✅ Motor: Forward")
	} else if direction == "backward" {
		forwardPin.Low()
		backwardPin.High()
		log.Println("✅ Motor: Backward")
	}
}

// StopMotor stops the car.
func StopMotor() {
	forwardPin.Low()
	backwardPin.Low()
	log.Println("🛑 Motor: Stopped")
}


