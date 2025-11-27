// File: /Omega-Code/servers/robot-controller-backend/gpio/gpio_real.go
package gpio

import "github.com/stianeikeland/go-rpio/v4"

// RealGPIO implements the GPIO interface for Raspberry Pi.
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

// RealGPIOPin wraps rpio.Pin to implement GPIOPin interface.
type RealGPIOPin rpio.Pin

func (p RealGPIOPin) Input() {
    rpio.Pin(p).Input()
}

func (p RealGPIOPin) Output() {
    rpio.Pin(p).Output()
}

// ✅ Convert `rpio.State` to `bool`
func (p RealGPIOPin) Read() bool {
    return rpio.Pin(p).Read() == rpio.High // ✅ Returns true if HIGH, false if LOW
}

func (p RealGPIOPin) High() {
    rpio.Pin(p).High()
}

func (p RealGPIOPin) Low() {
    rpio.Pin(p).Low()
}
