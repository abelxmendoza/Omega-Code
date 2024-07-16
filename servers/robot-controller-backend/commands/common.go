package commands

import (
	"bytes"
	"fmt"
	"log"
	"os/exec"
	"net/http"
)

var execCommand = exec.Command // Declare execCommand for mocking in tests

func logRequest(r *http.Request) {
	log.Printf("Received %s request for %s from %s\n", r.Method, r.URL, r.RemoteAddr)
}

func executePythonScript(scriptType, param1, param2 string) error {
	cmdArgs := []string{fmt.Sprintf("%s_control.py", scriptType), param1, param2}
	command := execCommand("python3", cmdArgs...)
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
