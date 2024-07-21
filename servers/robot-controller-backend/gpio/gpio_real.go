// File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_real.go

/*
Package gpio provides a real implementation of GPIO for Raspberry Pi hardware.
It uses the go-rpio library to interact with the GPIO pins.
*/

package gpio

import "github.com/stianeikeland/go-rpio/v4"

// RealGPIO implements the GPIO interface for actual Raspberry Pi hardware.
type RealGPIO struct{}

func (r RealGPIO) Open() error {
    return rpio.Open()
}

func (r RealGPIO) Close() error {
    return rpio.Close()
}

func (r RealGPIO) Pin(pin int) GPIOPin {
    return RealGPIOPin(rpio.Pin(pin))
}

// RealGPIOPin wraps the rpio.Pin type to implement the GPIOPin interface.
type RealGPIOPin rpio.Pin

func (p RealGPIOPin) Input() {
    rpio.Pin(p).Input()
}

func (p RealGPIOPin) Output() {
    rpio.Pin(p).Output()
}

func (p RealGPIOPin) Read() rpio.State {
    return rpio.Pin(p).Read()
}

func (p RealGPIOPin) High() {
    rpio.Pin(p).High()
}

func (p RealGPIOPin) Low() {
    rpio.Pin(p).Low()
}
