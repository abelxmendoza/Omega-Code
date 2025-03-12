/*
Package gpio provides a mock implementation of GPIO for testing purposes.
It simulates GPIO operations without requiring actual hardware.
*/

package gpio

import "log"

// MockGPIO implements the GPIO interface for testing purposes.
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
