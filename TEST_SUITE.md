# Omega-Code Test Suite

## Overview

Comprehensive test coverage for the Omega-1 robotics platform, including frontend UI, backend services, and integration tests.

## Test Statistics

- **Backend**: 569+ test cases
- **Frontend**: 828+ test cases
- **Coverage**: Critical paths at 100%, overall >80%

## Test Structure

### Frontend Tests (`ui/robot-controller-ui/tests/`)

#### Unit Tests
- Component tests: `Header`, `SystemModeDashboard`, `LatencyDashboard`, `LedModal`, `CarControlPanel`, `SpeedControl`
- Hook tests: `useLatencyMetrics`, `useRobotOnline`
- Integration tests: System mode integration, API integration

#### E2E Tests (Cypress)
- Robot lifecycle workflow
- Robot control workflow
- System mode workflow

#### Snapshot Tests
- `SystemModeDashboard` snapshot
- `LatencyDashboard` snapshot

### Backend Tests (`servers/robot_controller_backend/tests/`)

#### Unit Tests
- API routes: System mode, autonomy, lighting
- Video processing: Latency endpoints, hybrid system, ArUco detection, face recognition
- Movement: Motor control, servo control, straight drive assist
- Hardware: PID controllers, sensor fusion
- LED control: Pattern tests, control tests

#### Integration Tests
- System mode API integration
- Video server integration
- Latency integration
- Full system integration

#### E2E Tests
- Complete workflow tests
- Robot lifecycle tests
- Autonomous driving tests

#### Fault Injection Tests
- Orin failure scenarios
- Corrupted frame handling
- ROS bridge delay
- Frame drop recovery

#### Performance Tests
- Latency performance
- Load testing
- Performance benchmarks

#### Security Tests
- API security (SQL injection, XSS)
- Input validation

## Running Tests

### Frontend Tests

```bash
cd ui/robot-controller-ui
npm test                    # Run all tests
npm test -- --coverage      # With coverage
npm run test:e2e            # E2E tests
```

### Backend Tests

```bash
cd servers/robot_controller_backend
source venv/bin/activate
pytest tests/unit -v        # Unit tests
pytest tests/integration -v # Integration tests
pytest tests/ -v            # All tests
pytest --coverage           # With coverage
```

### All Tests

```bash
make test                   # Run all test suites
make test-coverage          # With coverage reports
bash scripts/run_all_tests_fixed.sh
```

## Test Coverage Goals

### Frontend
- Statements: >80%
- Branches: >75%
- Lines: >80%
- Functions: >75%

### Backend
- Statements: >85%
- Branches: >75%
- Lines: >85%
- Functions: >75%

## CI/CD Integration

Tests run automatically on:
- Push to `main`/`master` branches
- Pull requests
- Nightly schedule (2 AM UTC)
- Manual workflow dispatch

See [CI_CD_SETUP.md](CI_CD_SETUP.md) for details.

## Test Features

### Hardware-Independent Testing
- Hardware-dependent tests automatically skipped in CI
- Mock hardware interfaces for unit tests
- Graceful fallback when hardware unavailable

### Demo Mode Testing
- Frontend tests verify demo mode functionality
- UI controls work without hardware connection
- Simulated commands tested

### Integration Test Resilience
- Tests gracefully skip if services unavailable
- Automatic retry logic for flaky tests
- Clear error messages for debugging

## Key Test Areas

### ✅ Fully Tested
- System mode API routes
- Latency endpoints
- Hybrid system manager
- Video processing
- Movement V2 components
- Frontend components
- Security validation

### 🔄 Partially Tested
- ROS2 integration (structure exists, needs implementation)
- Visual regression tests (structure exists, needs refinement)

### 📋 Future Tests
- ROS2 node tests
- More comprehensive E2E scenarios
- Stress testing for hybrid system
- Memory leak tests
- Accessibility tests

## Test Maintenance

- Tests are updated alongside feature development
- Breaking changes require test updates
- Coverage reports track progress
- CI ensures tests pass before merge

---

**Last Updated**: 2025-01-XX
**Test Status**: ✅ Production Ready

