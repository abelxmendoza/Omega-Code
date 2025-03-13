// File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_mock.go

package gpio

import "log"

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
    return &MockGPIOPin{pin: pin}
}

// MockGPIOPin simulates GPIO pin behavior.
type MockGPIOPin struct {
    pin   int
    state rpio.State
}

func (p *MockGPIOPin) Input() {}

func (p *MockGPIOPin) Output() {}

func (p *MockGPIOPin) Read() rpio.State {
    return p.state
}

func (p *MockGPIOPin) High() {
    log.Printf("⚡ [MOCK] Pin %d set to HIGH\n", p.pin)
    p.state = High
}

func (p *MockGPIOPin) Low() {
    log.Printf("⚡ [MOCK] Pin %d set to LOW\n", p.pin)
    p.state = Low
}

func (p *MockGPIOPin) SetState(state rpio.State) { // ✅ Implemented SetState
    p.state = state
    log.Printf("⚡ [MOCK] Pin %d set to state %v\n", p.pin, state)
}
