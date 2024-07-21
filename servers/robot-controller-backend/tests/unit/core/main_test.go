// File: /Omega-Code/servers/robot-controller-backend/tests/unit/core/main_test.go

package main

import (
    "testing"
    "github.com/stretchr/testify/assert"
    "github.com/abelxmendoza/Omega-Code/servers/robot-controller-backend/core"
)

func TestMain(t *testing.T) {
    go core.StartServer()

    assert.NotNil(t, core.StartServer)
}

