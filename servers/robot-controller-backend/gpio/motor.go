/*
# File: /Omega-Code/servers/robot-controller-backend/gpio/motor.go
# Summary:
Controls the car's motors for forward and backward movement using GPIO.
*/

package gpio

import (
	"log"

	"github.com/stianeikeland/go-rpio/v4"
)

// MotorController wraps GPIO for motor operations.
type MotorController struct {
	ForwardPin  rpio.Pin
	BackwardPin rpio.Pin
}

// InitMotor initializes the motor controller.
func InitMotor() *MotorController {
	return &MotorController{
		ForwardPin:  rpio.Pin(19),
		BackwardPin: rpio.Pin(20),
	}
}

// ActivateMotor moves the car forward or backward.
func (m *MotorController) ActivateMotor(direction string) {
	if direction == "forward" {
		m.ForwardPin.Output()
		m.BackwardPin.Output()
		m.ForwardPin.High()
		m.BackwardPin.Low()
		log.Println("✅ Motor: Forward")
	} else if direction == "backward" {
		m.ForwardPin.Output()
		m.BackwardPin.Output()
		m.ForwardPin.Low()
		m.BackwardPin.High()
		log.Println("✅ Motor: Backward")
	}
}

// StopMotor stops the car.
func (m *MotorController) StopMotor() {
	m.ForwardPin.Output()
	m.BackwardPin.Output()
	m.ForwardPin.Low()
	m.BackwardPin.Low()
	log.Println("🛑 Motor: Stopped")
}
