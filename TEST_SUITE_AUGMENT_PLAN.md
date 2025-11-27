# OMEGA HYBRID ROBOTICS TEST SUITE — AUGMENTATION BLUEPRINT

This document outlines the comprehensive augmentation plan for the Omega Robot Controller test suite, focusing on hybrid system testing, fault injection, and complete lifecycle coverage.

## Table of Contents

1. [Hybrid Mode Switching Tests](#1-hybrid-mode-switching-tests)
2. [Fault Injection Tests](#2-fault-injection-tests)
3. [Robot Lifecycle E2E Tests](#3-robot-lifecycle-e2e-tests)
4. [Fake Orin Mock Server](#4-fake-orin-mock-server)
5. [ROS2 Node Test Harness](#5-ros2-node-test-harness)
6. [Snapshot Visual Tests](#6-snapshot-visual-tests)
7. [CI Test Matrix](#7-ci-test-matrix)
8. [Coverage Goals](#8-coverage-goals)
9. [TODO Tracking](#9-todo-tracking)

---

## 1. Hybrid Mode Switching Tests

### Goals
- Verify mode switching between PI_ONLY, PI_ORIN_HYBRID, and ORIN_ONLY
- Test switching under load
- Verify WebSocket push events for "mode_update"
- Ensure thermal & CPU watchdog can force mode changes

### Test Files
- `servers/robot-controller-backend/tests/hybrid/test_hybrid_mode_switching.py`
- `servers/robot-controller-backend/tests/hybrid/test_mode_ws_updates.py`
- `servers/robot-controller-backend/tests/hybrid/test_thermal_to_mode_transition.py`

---

## 2. Fault Injection Tests

### Goals
- Simulate ORIN_FAILURE scenarios
- Test corrupted frame handling
- Test ROS bridge delays
- Test frame drop recovery

### Test Files
- `servers/robot-controller-backend/tests/faults/test_orin_failure.py`
- `servers/robot-controller-backend/tests/faults/test_pi_corrupted_frames.py`
- `servers/robot-controller-backend/tests/faults/test_ros_bridge_delay.py`
- `servers/robot-controller-backend/tests/faults/test_frame_drop_recovery.py`

---

## 3. Robot Lifecycle E2E Tests

### Goals
- Test complete robot boot → operation → shutdown workflow
- Verify UI responsiveness under load
- Test mode transitions
- Verify WebSocket stability

### Test Files
- `ui/robot-controller-ui/cypress/e2e/robot-lifecycle.cy.ts`

---

## 4. Fake Orin Mock Server

### Goals
- Mock Orin brain for testing without hardware
- Simulate various latency scenarios
- Test failure modes

### Files
- `servers/robot-controller-backend/tests/mocks/orin_mock/orin_mock_server.py`
- `servers/robot-controller-backend/tests/mocks/orin_mock/messages.py`
- `servers/robot-controller-backend/tests/mocks/orin_mock/fake_inference.py`

---

## 5. ROS2 Node Test Harness

### Goals
- Test ROS2 node integration
- Test message passing
- Test round-trip latency

### Files
- `ros/test_harness/launch_test_nodes.py`
- `ros/test_harness/ros_test_runner.py`
- `ros/test_harness/test_pi_sensor_hub.py`
- `ros/test_harness/test_orin_ai_brain.py`
- `ros/test_harness/test_round_trip_latency.py`

---

## 6. Snapshot Visual Tests

### Goals
- Detect visual regressions
- Test UI component rendering

### Files
- `ui/robot-controller-ui/tests/snapshots/SystemModeDashboard.snapshot.test.tsx`
- `ui/robot-controller-ui/tests/snapshots/LatencyDashboard.snapshot.test.tsx`

---

## 7. CI Test Matrix

### Goals
- Run tests across multiple platforms
- Test multiple Python/Node versions

---

## 8. Coverage Goals

- Frontend: 80% statements, 75% branches, 80% lines
- Backend: 85% statements, 75% branches, 85% lines
- ROS2: 70% overall, 90% message/event coverage

---

## 9. TODO Tracking

Track TODOs in:
- `video_server.py`
- `hybrid_system.py`
- `orin_ai_brain.py`
- `pi_sensor_hub.py`
- `websocket_server.py`

