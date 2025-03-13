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
    return &MockGPIOPin{pin: pin, state: false} // ✅ Default state set to false (LOW)
}

// MockGPIOPin simulates GPIO pin behavior.
type MockGPIOPin struct {
    pin   int
    state bool // ✅ Changed from `rpio.State` to `bool`
}

func (p *MockGPIOPin) Input() {}

func (p *MockGPIOPin) Output() {}

func (p *MockGPIOPin) Read() bool { // ✅ Changed return type to `bool`
    return p.state
}

func (p *MockGPIOPin) High() {
    log.Printf("⚡ [MOCK] Pin %d set to HIGH\n", p.pin)
    p.state = true // ✅ High is now `true`
}

func (p *MockGPIOPin) Low() {
    log.Printf("⚡ [MOCK] Pin %d set to LOW\n", p.pin)
    p.state = false // ✅ Low is now `false`
}

func (p *MockGPIOPin) SetState(state bool) { // ✅ Added `SetState()` implementation
    p.state = state
    log.Printf("⚡ [MOCK] Pin %d set to state %v\n", p.pin, state)
}
