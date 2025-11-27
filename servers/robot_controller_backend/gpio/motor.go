// File: /Omega-Code/servers/robot-controller-backend/gpio/motor.go

package gpio

import "log"

// MotorController wraps GPIO for motor operations.
type MotorController struct {
    ForwardPin  GPIOPin
    BackwardPin GPIOPin
}

// InitMotor initializes the motor controller.
func InitMotor() *MotorController {
    return &MotorController{
        ForwardPin:  GpioInterface.Pin(19), // ✅ Fixed reference
        BackwardPin: GpioInterface.Pin(20), // ✅ Fixed reference
    }
}

// ActivateMotor moves the car forward or backward.
func (m *MotorController) ActivateMotor(direction string) {
    if direction == "forward" {
        m.ForwardPin.High()
        m.BackwardPin.Low()
        log.Println("✅ Motor: Forward")
    } else if direction == "backward" {
        m.ForwardPin.Low()
        m.BackwardPin.High()
        log.Println("✅ Motor: Backward")
    }
}

// StopMotor stops the car.
func (m *MotorController) StopMotor() {
    m.ForwardPin.Low()
    m.BackwardPin.Low()
    log.Println("🛑 Motor: Stopped")
}
