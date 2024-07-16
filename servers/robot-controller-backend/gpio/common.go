// gpio/common.go

package gpio

import "github.com/stianeikeland/go-rpio/v4"

type GPIO interface {
    Open() error
    Close() error
    Pin(int) GPIOPin
}

type GPIOPin interface {
    Input()
    Output()
    Read() rpio.State
    High()
    Low()
}

var (
    Low  rpio.State = rpio.Low
    High rpio.State = rpio.High
)
