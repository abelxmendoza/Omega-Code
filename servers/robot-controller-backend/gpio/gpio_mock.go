// File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_mock.go

package gpio

import (
	"log"

	"github.com/stianeikeland/go-rpio/v4" // ✅ Import rpio for consistent types
)

// MockGPIO implements the GPIO interface for testing.
type MockGPIO struct{}

func (m MockGPIO) Open() error {
	log.Println("⚡ [MOCK] GPIO Opened")
	return nil
}

func (m MockGPIO) Close() error {
	log.Println("⚡ [MOCK] GPIO Closed")
	return nil
}

func (m MockGPIO) Pin(pin int) GPIOPin {
	return &MockGPIOPin{pin: pin, state: rpio.Low} // ✅ Default state set to rpio.Low
}

// MockGPIOPin simulates GPIO pin behavior.
type MockGPIOPin struct {
	pin   int
	state rpio.State // ✅ Changed from `bool` to `rpio.State`
}

func (p *MockGPIOPin) Input() {}

func (p *MockGPIOPin) Output() {}

func (p *MockGPIOPin) Read() rpio.State { // ✅ Fixed return type
	return p.state
}

func (p *MockGPIOPin) High() {
	log.Printf("⚡ [MOCK] Pin %d set to HIGH\n", p.pin)
	p.state = rpio.High // ✅ High is now `rpio.High`
}

func (p *MockGPIOPin) Low() {
	log.Printf("⚡ [MOCK] Pin %d set to LOW\n", p.pin)
	p.state = rpio.Low // ✅ Low is now `rpio.Low`
}
