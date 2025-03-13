// File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_mock.go

package gpio

import (
	"log"

	"github.com/stianeikeland/go-rpio/v4" // ✅ Ensure rpio is imported for compatibility
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
	return &MockGPIOPin{pin: pin, state: false} // ✅ Use bool for simulation
}

// MockGPIOPin simulates GPIO pin behavior.
type MockGPIOPin struct {
	pin   int
	state bool // ✅ Use bool for mock simulation
}

func (p *MockGPIOPin) Input() {}

func (p *MockGPIOPin) Output() {}

func (p *MockGPIOPin) Read() rpio.State { // ✅ Convert bool → rpio.State for compatibility
	if p.state {
		return rpio.High
	}
	return rpio.Low
}

func (p *MockGPIOPin) High() {
	log.Printf("⚡ [MOCK] Pin %d set to HIGH\n", p.pin)
	p.state = true
}

func (p *MockGPIOPin) Low() {
	log.Printf("⚡ [MOCK] Pin %d set to LOW\n", p.pin)
	p.state = false
}

// ✅ Compatibility function (optional) if needed for real GPIO behavior
func (p *MockGPIOPin) SetState(state bool) {
	p.state = state
	log.Printf("⚡ [MOCK] Pin %d manually set to %v\n", p.pin, state)
}
