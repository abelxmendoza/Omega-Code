# CI/CD Pipeline Documentation

## Overview

Comprehensive CI/CD pipeline for Omega-Code using GitHub Actions. Automated testing, building, and deployment for both frontend and backend components.

## Workflows

### 1. Main CI Pipeline (`.github/workflows/ci.yml`)

**Triggers:**
- Push to `main`, `master`, `develop` branches
- Pull requests to `main`, `master`, `develop` branches
- Manual workflow dispatch

**Jobs:**
- **Frontend Tests**: Jest unit tests with coverage
- **Backend Tests**: Pytest unit tests (Python 3.10, 3.11)
- **Integration Tests**: Integration tests (gracefully skips if services unavailable)
- **Security Tests**: npm audit, bandit, safety scans
- **Test Summary**: Aggregated test results

### 2. Comprehensive Test Suite (`.github/workflows/test-suite.yml`)

**Triggers:**
- Push/PR to main branches
- Nightly schedule (2 AM UTC)
- Manual dispatch

**Jobs:**
- **Test Matrix**: Multi-platform (Ubuntu, macOS) with Node.js 18/20 and Python 3.10/3.11
- **Frontend Unit Tests**: Jest with coverage upload
- **Frontend E2E Tests**: Cypress E2E tests
- **Backend Unit Tests**: Pytest with coverage (Python 3.9/3.10/3.11)
- **Backend Integration Tests**: Integration tests with Redis service
- **Security Tests**: Security scanning and audits
- **Performance Tests**: Performance benchmarks

### 3. Frontend CI/CD (`.github/workflows/frontend-ci-cd.yml`)

**Triggers:**
- Push to `master`, `main`, `develop` (frontend paths only)
- Pull requests to `master`, `main` (frontend paths only)
- Manual dispatch

**Jobs:**
- **Lint & Type Check**: ESLint, TypeScript type checking
- **Tests**: Unit tests with coverage
- **Build**: Next.js production build
- **Security Scan**: npm audit, Snyk (if configured)
- **E2E Tests**: Cypress end-to-end tests
- **Deploy to Vercel**: Automatic deployment (if configured)

### 4. Backend CI/CD (`.github/workflows/backend-ci-cd.yml`)

**Triggers:**
- Push to `master`, `main`, `develop` (backend paths only)
- Pull requests to `master`, `main` (backend paths only)
- Manual dispatch

**Jobs:**
- **Python Tests**: Multi-version testing (3.9, 3.10, 3.11)
- **Go Tests**: Multi-version testing (1.20, 1.21, 1.22) for lighting server
- **Lint & Code Quality**: Black, flake8, bandit (if configured)
- **Security Scan**: Bandit, safety checks
- **Integration Tests**: Full system integration testing
- **Build Docker Images**: Docker builds (if configured)
- **Deploy**: Staging/production deployment (if configured)

## Test Coverage

### Backend
- **Unit Tests**: 569+ test cases
- **Integration Tests**: 5+ test suites
- **E2E Tests**: 4+ complete workflows
- **Fault Injection**: 4+ resilience tests
- **Performance Tests**: 3+ benchmark suites
- **Security Tests**: 2+ security validation suites
- **Hybrid System**: 3+ Pi+Jetson integration tests

### Frontend
- **Unit Tests**: 828+ test cases
- **Integration Tests**: 2+ component integration tests
- **E2E Tests**: 4+ Cypress browser tests
- **Snapshot Tests**: 2+ visual regression tests
- **Performance Tests**: Latency performance measurements

## Running Tests Locally

### Frontend
```bash
cd ui/robot-controller-ui
npm test                    # Unit tests
npm run test:e2e           # E2E tests (if configured)
npm test -- --coverage     # With coverage
```

### Backend
```bash
cd servers/robot_controller_backend
source venv/bin/activate
pytest tests/unit -v                    # Unit tests
pytest tests/integration -v             # Integration tests
pytest tests/e2e -v                     # E2E tests
pytest tests/ -v -m "not hardware"      # All non-hardware tests
```

### All Tests
```bash
make test                   # Run all test suites
bash scripts/run_all_tests_fixed.sh
```

## CI/CD Features

### ✅ Automated Testing
- Runs on every push and PR
- Multi-platform testing (Ubuntu, macOS)
- Multi-version testing (Node.js 18/20, Python 3.9/3.10/3.11)
- Hardware-dependent tests automatically skipped

### ✅ Coverage Reports
- Codecov integration for coverage tracking
- Coverage reports uploaded automatically
- Coverage goals: Frontend >80%, Backend >85%

### ✅ Security Scanning
- npm audit for frontend dependencies
- Bandit for Python security scanning
- Safety for Python dependency vulnerabilities
- Security tests for API validation

### ✅ Performance Testing
- Performance benchmarks
- Load testing capabilities
- Latency measurements

### ✅ Graceful Degradation
- Integration tests skip if services unavailable
- Hardware tests automatically skipped in CI
- Tests marked with appropriate markers for filtering

## Configuration

### Required Secrets (Optional)

#### Frontend
- `VERCEL_TOKEN`: Vercel deployment token (for auto-deploy)
- `VERCEL_ORG_ID`: Vercel organization ID
- `VERCEL_PROJECT_ID`: Vercel project ID

#### Backend
- `DOCKER_USERNAME`: Docker Hub username (for image publishing)
- `DOCKER_PASSWORD`: Docker Hub password

### Environment Variables

Tests run with default environment variables. Hardware-dependent features are automatically mocked or skipped.

## Test Markers

### Backend Markers
- `hardware`: Hardware-dependent tests (skipped in CI)
- `integration`: Integration tests
- `e2e`: End-to-end tests
- `security`: Security tests
- `performance`: Performance tests

### Usage
```bash
pytest tests/ -v -m "not hardware"        # Skip hardware tests
pytest tests/ -v -m "integration"          # Only integration tests
pytest tests/ -v -m "e2e"                 # Only E2E tests
```

## Troubleshooting

### Common Issues

#### Tests Failing in CI
1. Check GitHub Actions logs for detailed errors
2. Verify test markers are correct (`-m "not hardware"`)
3. Ensure dependencies are installed correctly
4. Check if services are available for integration tests

#### Coverage Not Uploading
1. Verify Codecov token is configured (optional)
2. Check coverage file paths match workflow configuration
3. Ensure `fail_ci_if_error: false` is set for optional coverage

#### E2E Tests Failing
1. Verify Cypress is configured in `package.json`
2. Check if backend services need to be running
3. Review Cypress screenshots/videos in artifacts

## Best Practices

1. **Test Locally First**: Run tests locally before pushing
2. **Use Test Markers**: Mark hardware-dependent tests appropriately
3. **Graceful Skipping**: Tests should skip gracefully if dependencies unavailable
4. **Coverage Goals**: Maintain >80% coverage for critical paths
5. **Security First**: Always run security scans before merging

## Status

✅ **CI/CD Pipeline**: Fully operational  
✅ **Test Coverage**: Comprehensive (1,397+ test cases)  
✅ **Multi-Platform**: Ubuntu + macOS support  
✅ **Multi-Version**: Multiple Node.js and Python versions  
✅ **Security Scanning**: Automated security audits  
✅ **Performance Testing**: Benchmark suites included  

---

**Last Updated**: 2025-01-XX  
**Status**: ✅ Production Ready

