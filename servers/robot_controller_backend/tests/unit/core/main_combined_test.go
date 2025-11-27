// File: /Omega-Code/servers/robot-controller-backend/tests/unit/core/main_combined_test.go

package main

import (
    "testing"
    "github.com/stretchr/testify/assert"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/core"
    "github.com/abelxmendoza/Omega-Code/servers/robot_controller_backend/commands"
)

func TestMain(t *testing.T) {
    go core.StartPythonVideoServer()
    go commands.StartROSNodes()
    go core.StartServer()

    assert.NotNil(t, core.StartPythonVideoServer)
    assert.NotNil(t, commands.StartROSNodes)
    assert.NotNil(t, core.StartServer)
}

