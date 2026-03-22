# Testing Guide — Omega-Code

## Overview

| Layer | Tests | Coverage Goal |
|-------|-------|---------------|
| Backend | 600+ test cases | >85% statements |
| Frontend | 850+ test cases | >80% statements |
| OMEGAOS components | 63+ test cases | Critical paths 100% |

---

## Running Tests

### Frontend

```bash
cd ui/robot-controller-ui

npm test                    # All unit tests
npm test -- --coverage      # With coverage report
npm run test:e2e            # Cypress E2E tests
```

### Backend

```bash
cd servers/robot_controller_backend
source venv/bin/activate

pytest tests/unit -v                     # Unit tests
pytest tests/integration -v              # Integration tests
pytest tests/e2e -v                      # E2E tests
pytest tests/ -v -m "not hardware"       # All non-hardware tests
pytest tests/ --coverage                 # With coverage
```

### All tests at once

```bash
make test
make test-coverage
bash scripts/run_all_tests_fixed.sh
```

---

## Test Structure

### Frontend (`ui/robot-controller-ui/tests/`)

**Unit tests (Jest):**
- Components: `Header`, `SystemModeDashboard`, `LatencyDashboard`, `LedModal`, `CarControlPanel`, `SpeedControl`
- Hooks: `useLatencyMetrics`, `useRobotOnline`
- Settings: `ProfileSelector`, `SettingsPage`

**Integration tests:**
- System mode integration
- API integration

**E2E (Cypress):**
- Robot lifecycle workflow
- Robot control workflow
- System mode workflow

**Snapshot tests:**
- `SystemModeDashboard`
- `LatencyDashboard`

### Backend (`servers/robot_controller_backend/tests/`)

**Unit tests:**
- API routes: system mode, autonomy, lighting
- Video: latency endpoints, hybrid system, ArUco, face recognition
- Movement: motor control, servo control, straight drive assist
- Hardware: PID controllers, sensor fusion
- LED: pattern tests, control tests
- OMEGAOS: Service Orchestrator, Configuration Layer, Settings UI

**Integration tests:**
- System mode API
- Video server
- Latency
- Full system
- Service-Config integration

**E2E tests:**
- Complete workflow
- Robot lifecycle
- Autonomous driving

**Fault injection tests:**
- Orin failure scenarios
- Corrupted frame handling
- ROS bridge delay
- Frame drop recovery

**Performance tests:**
- Latency benchmarks
- Load testing

**Security tests:**
- SQL injection / XSS prevention
- Input validation

---

## Test Markers (Backend)

```bash
pytest tests/ -v -m "not hardware"    # Skip hardware-dependent tests (CI-safe)
pytest tests/ -v -m "integration"     # Only integration tests
pytest tests/ -v -m "e2e"            # Only E2E tests
pytest tests/ -v -m "security"       # Only security tests
pytest tests/ -v -m "performance"    # Only performance tests
```

Hardware-dependent tests are automatically skipped in CI — mark them with `@pytest.mark.hardware`.

---

## Coverage Goals

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

---

## Key Test Areas

### Fully tested
- System mode API routes
- Latency endpoints
- Hybrid system manager
- Video processing
- Movement V2 components
- Frontend components
- Security validation
- Service Orchestrator
- Configuration Layer

### Partially tested
- ROS2 integration (structure exists, expanding)

---

## CI Integration

Tests run automatically on every push and PR. See [CI_CD.md](CI_CD.md) for workflow details.

Hardware tests are automatically skipped in CI. Integration tests skip gracefully if services are unavailable.
