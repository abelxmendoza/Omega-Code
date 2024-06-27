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
    return MockGPIOPin{}
}

type MockGPIOPin struct{}

func (p MockGPIOPin) Input() {}

func (p MockGPIOPin) Output() {}

func (p MockGPIOPin) Read() rpio.State {
    return Low // Mock read to return Low state
}

func (p MockGPIOPin) High() {}

func (p MockGPIOPin) Low() {}
