package main

/*
#cgo LDFLAGS: -L./rust_module/target/release -lrust_module
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
    output := C.GoString(cOutput)
    C.free(unsafe.Pointer(cOutput)) // Ensure this is only called once
    return output
}

func processLineTrackingData(input string) string {
    cInput := C.CString(input)
    defer C.free(unsafe.Pointer(cInput))

    cOutput := C.process_line_tracking_data(cInput)
    output := C.GoString(cOutput)
    C.free(unsafe.Pointer(cOutput)) // Ensure this is only called once
    return output
}
