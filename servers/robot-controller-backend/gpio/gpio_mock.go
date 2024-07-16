package main

import (
    "github.com/stianeikeland/go-rpio/v4"
)

// MockGPIO implements the GPIO interface for testing purposes
type MockGPIO struct{}

func (m MockGPIO) Open() error {
    return nil
}

func (m MockGPIO) Close() error {
    return nil
}

func (m MockGPIO) Pin(pin int) GPIOPin {
    return &MockGPIOPin{pin: pin}
}

type MockGPIOPin struct {
    pin   int
    state rpio.State
}

func (p *MockGPIOPin) Input() {}

func (p *MockGPIOPin) Output() {}

func (p *MockGPIOPin) Read() rpio.State {
    // Return specific values based on the pin number for testing
    switch p.pin {
    case 14:
        return High // Mock read to return High state for pin 14
    case 15:
        return Low  // Mock read to return Low state for pin 15
    case 23:
        return High // Mock read to return High state for pin 23
    case 22:
        // Simulate the echo pin behavior
        if p.state == Low {
            p.state = High
            return Low
        }
        return High
    default:
        return Low
    }
}

func (p *MockGPIOPin) High() {
    p.state = High
}

func (p *MockGPIOPin) Low() {
    p.state = Low
}

