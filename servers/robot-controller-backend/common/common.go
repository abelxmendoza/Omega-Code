// File: /Omega-Code/servers/robot-controller-backend/common/common.go

package common

import (
    "bytes"
    "log"
    "net/http"
    "os/exec"
)

// ExecutePythonScript runs a Python script with given parameters
func ExecutePythonScript(scriptType, param1, param2, param3, param4 string) error {
    cmdArgs := []string{fmt.Sprintf("%s_control.py", scriptType), param1, param2, param3, param4}
    command := exec.Command("python3", cmdArgs...)
    var out bytes.Buffer
    var stderr bytes.Buffer
    command.Stdout = &out
    command.Stderr = &stderr
    err := command.Run()
    if err != nil {
        log.Printf("Error executing Python script: %s\n", stderr.String())
        return err
    }
    log.Printf("Python script output: %s\n", out.String())
    return nil
}

// LogRequest logs the details of an HTTP request
func LogRequest(r *http.Request) {
    log.Printf("Received %s request for %s from %s\n", r.Method, r.URL, r.RemoteAddr)
}

