# Comprehensive Test Suite Documentation

This document describes the complete test suite for the Omega Robot Controller application, covering unit tests, integration tests, end-to-end tests, security tests, and performance tests.

## Table of Contents

1. [Test Structure](#test-structure)
2. [Running Tests](#running-tests)
3. [Test Types](#test-types)
4. [Test Coverage](#test-coverage)
5. [CI/CD Integration](#cicd-integration)
6. [Writing New Tests](#writing-new-tests)

---

## Test Structure

### Frontend Tests (`ui/robot-controller-ui/`)

```
ui/robot-controller-ui/
├── tests/
│   ├── unit/              # Component unit tests
│   ├── integration/       # Component integration tests
│   ├── e2e/              # End-to-end tests (Cypress)
│   ├── security/         # Security tests
│   ├── performance/      # Performance tests
│   └── utils/            # Test utilities and mocks
├── cypress/
│   ├── e2e/              # Cypress E2E specs
│   ├── fixtures/          # Test fixtures
│   └── support/          # Cypress support files
└── jest.config.js         # Jest configuration
```

### Backend Tests (`servers/robot-controller-backend/tests/`)

```
servers/robot-controller-backend/tests/
├── unit/                  # Unit tests
│   ├── api/              # API route tests
│   ├── controllers/      # Controller tests
│   ├── video/            # Video processing tests
│   ├── movement/         # Movement control tests
│   ├── autonomy/         # Autonomy controller tests
│   └── hardware/         # Hardware abstraction tests
├── integration/           # Integration tests
│   ├── api/              # API integration tests
│   ├── video/            # Video server integration
│   └── system/           # System integration tests
├── e2e/                  # End-to-end tests
│   ├── workflows/        # Full workflow tests
│   └── scenarios/        # Scenario-based tests
├── security/             # Security tests
│   ├── api/              # API security tests
│   ├── auth/             # Authentication tests
│   └── input/            # Input validation tests
├── performance/          # Performance tests
│   ├── load/             # Load tests
│   ├── stress/           # Stress tests
│   └── benchmarks/       # Benchmark tests
└── utils/                # Test utilities
    ├── fixtures/         # Test fixtures
    ├── mocks/            # Mock objects
    └── helpers/          # Helper functions
```

### ROS2 Tests (`ros/tests/`)

```
ros/tests/
├── unit/                  # ROS2 node unit tests
├── integration/           # ROS2 integration tests
└── e2e/                  # ROS2 E2E tests
```

---

## Running Tests

### Frontend Tests

```bash
# Unit tests
cd ui/robot-controller-ui
npm test                    # Run all tests
npm test -- --watch        # Watch mode
npm test -- --coverage     # With coverage

# E2E tests
npm run test:e2e          # Headless
npm run test:e2e:open     # Interactive

# Type checking
npm run type-check

# Linting
npm run lint

# Security audit
npm run security:audit
npm run security:scan
```

### Backend Tests

```bash
# Activate virtual environment
cd servers/robot-controller-backend
source venv/bin/activate  # or your venv path

# Unit tests
pytest tests/unit -v
pytest tests/unit/api -v
pytest tests/unit/video -v

# Integration tests
pytest tests/integration -v

# E2E tests
pytest tests/e2e -v

# Security tests
pytest tests/security -v

# Performance tests
pytest tests/performance -v

# All tests with coverage
pytest --cov=. --cov-report=html

# Specific test file
pytest tests/unit/api/test_autonomy_routes.py -v
```

### ROS2 Tests

```bash
cd ros
colcon test --packages-select omega_robot
colcon test-result --verbose
```

### Run All Tests

```bash
# From project root
make test                  # Run all test suites
make test-frontend         # Frontend only
make test-backend          # Backend only
make test-ros              # ROS2 only
make test-coverage         # With coverage reports
```

---

## Test Types

### 1. Unit Tests

**Purpose**: Test individual components in isolation

**Frontend Examples**:
- Component rendering
- Hook behavior
- Utility functions
- State management

**Backend Examples**:
- Function logic
- Class methods
- Data transformations
- Business logic

### 2. Integration Tests

**Purpose**: Test component interactions

**Frontend Examples**:
- Component composition
- API client integration
- WebSocket connections
- State management flows

**Backend Examples**:
- API endpoint integration
- Database interactions
- Service communication
- External API calls

### 3. End-to-End Tests

**Purpose**: Test complete user workflows

**Examples**:
- Robot control flow
- Camera feed display
- System mode switching
- Network configuration
- Autonomy mode activation

### 4. Security Tests

**Purpose**: Test security vulnerabilities

**Examples**:
- Input validation
- Authentication/authorization
- SQL injection prevention
- XSS prevention
- CSRF protection
- Rate limiting

### 5. Performance Tests

**Purpose**: Test performance and scalability

**Examples**:
- Load testing
- Stress testing
- Latency measurement
- Memory leaks
- CPU usage

---

## Test Coverage

### Coverage Goals

- **Unit Tests**: >80% coverage
- **Integration Tests**: >70% coverage
- **Critical Paths**: 100% coverage

### Viewing Coverage

```bash
# Frontend
cd ui/robot-controller-ui
npm test -- --coverage
open coverage/lcov-report/index.html

# Backend
cd servers/robot-controller-backend
pytest --cov=. --cov-report=html
open htmlcov/index.html
```

---

## CI/CD Integration

Tests run automatically on:
- Pull requests
- Pushes to main/master
- Scheduled runs (nightly)

See `.github/workflows/` for CI/CD configuration.

---

## Writing New Tests

### Frontend Test Example

```typescript
// tests/unit/components/Button.test.tsx
import { render, screen, fireEvent } from '@testing-library/react';
import Button from '@/components/Button';

describe('Button', () => {
  it('renders correctly', () => {
    render(<Button>Click me</Button>);
    expect(screen.getByText('Click me')).toBeInTheDocument();
  });

  it('handles click events', () => {
    const handleClick = jest.fn();
    render(<Button onClick={handleClick}>Click me</Button>);
    fireEvent.click(screen.getByText('Click me'));
    expect(handleClick).toHaveBeenCalledTimes(1);
  });
});
```

### Backend Test Example

```python
# tests/unit/api/test_autonomy_routes.py
import pytest
from fastapi.testclient import TestClient
from main_api import app

client = TestClient(app)

def test_start_autonomy_mode():
    response = client.post("/api/autonomy/start", json={"mode": "follow"})
    assert response.status_code == 200
    assert response.json()["ok"] is True
```

### E2E Test Example

```typescript
// cypress/e2e/robot-control.cy.ts
describe('Robot Control', () => {
  it('can control robot movement', () => {
    cy.visit('/');
    cy.get('[data-testid="move-up"]').click();
    cy.get('[data-testid="status"]').should('contain', 'Moving');
  });
});
```

---

## Test Best Practices

1. **Isolation**: Each test should be independent
2. **Naming**: Use descriptive test names
3. **Arrange-Act-Assert**: Follow AAA pattern
4. **Mocking**: Mock external dependencies
5. **Cleanup**: Clean up after tests
6. **Speed**: Keep tests fast
7. **Reliability**: Avoid flaky tests
8. **Documentation**: Document complex tests

---

## Troubleshooting

### Common Issues

1. **Tests failing in CI but passing locally**
   - Check environment variables
   - Verify test data availability
   - Check timing issues

2. **Slow tests**
   - Use mocks instead of real services
   - Parallelize test execution
   - Optimize test setup/teardown

3. **Flaky tests**
   - Add proper waits/timeouts
   - Use deterministic test data
   - Avoid race conditions

---

## Additional Resources

- [Jest Documentation](https://jestjs.io/docs/getting-started)
- [Cypress Documentation](https://docs.cypress.io/)
- [Pytest Documentation](https://docs.pytest.org/)
- [Testing Library Documentation](https://testing-library.com/)

