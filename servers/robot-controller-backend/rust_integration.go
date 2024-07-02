package main

/*
#cgo LDFLAGS: -L/Users/abel_elreaper/Desktop/Omega-Code/servers/robot-controller-backend/rust_module/target/release -lrust_module
#include <stdlib.h>

extern char* process_ultrasonic_data(char* input);
extern char* process_line_tracking_data(char* input);
*/
import "C"
import (
    "unsafe"
)

func processUltrasonicData(input string) string {
    cInput := C.CString(input)
    defer C.free(unsafe.Pointer(cInput))

    cOutput := C.process_ultrasonic_data(cInput)
    defer C.free(unsafe.Pointer(cOutput))

    return C.GoString(cOutput)
}

func processLineTrackingData(input string) string {
    cInput := C.CString(input)
    defer C.free(unsafe.Pointer(cInput))

    cOutput := C.process_line_tracking_data(cInput)
    defer C.free(unsafe.Pointer(cOutput))

    return C.GoString(cOutput)
}
