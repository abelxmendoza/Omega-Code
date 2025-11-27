# Omega Robot Controller - Makefile
# Comprehensive test and build automation

.PHONY: help test test-frontend test-backend test-ros test-all test-coverage test-security test-performance test-e2e lint format clean install install-frontend install-backend setup-venv

# Default target
help:
	@echo "Omega Robot Controller - Available Commands:"
	@echo ""
	@echo "Testing:"
	@echo "  make test              - Run all test suites"
	@echo "  make test-frontend     - Run frontend tests only"
	@echo "  make test-backend      - Run backend tests only"
	@echo "  make test-ros          - Run ROS2 tests only"
	@echo "  make test-coverage     - Run tests with coverage"
	@echo "  make test-security     - Run security tests"
	@echo "  make test-performance  - Run performance tests"
	@echo "  make test-e2e          - Run E2E tests"
	@echo ""
	@echo "Code Quality:"
	@echo "  make lint              - Run all linters"
	@echo "  make format            - Format all code"
	@echo ""
	@echo "Installation:"
	@echo "  make install           - Install all dependencies"
	@echo "  make install-frontend  - Install frontend dependencies"
	@echo "  make install-backend   - Install backend dependencies"
	@echo ""
	@echo "Setup:"
	@echo "  make setup-venv        - Setup Python virtual environment"
	@echo ""
	@echo "Cleanup:"
	@echo "  make clean             - Clean build artifacts"

# Test targets
test: test-frontend test-backend test-ros
	@echo "✅ All tests completed"

test-frontend:
	@echo "🧪 Running frontend tests..."
	cd ui/robot-controller-ui && npm test -- --watchAll=false

test-backend: setup-venv
	@echo "🧪 Running backend tests..."
	cd servers/robot_controller_backend && \
		source venv/bin/activate && \
		pytest tests/ -v

test-ros:
	@echo "🧪 Running ROS2 tests..."
	cd ros && colcon test --packages-select omega_robot && colcon test-result --verbose

test-coverage: test-coverage-frontend test-coverage-backend
	@echo "✅ Coverage reports generated"

test-coverage-frontend:
	@echo "📊 Generating frontend coverage..."
	cd ui/robot-controller-ui && npm test -- --coverage --watchAll=false

test-coverage-backend: setup-venv
	@echo "📊 Generating backend coverage..."
	cd servers/robot_controller_backend && \
		source venv/bin/activate && \
		pytest --cov=. --cov-report=html --cov-report=term

test-security: test-security-frontend test-security-backend
	@echo "✅ Security tests completed"

test-security-frontend:
	@echo "🔒 Running frontend security tests..."
	cd ui/robot-controller-ui && \
		npm run security:audit && \
		npm run security:scan

test-security-backend: setup-venv
	@echo "🔒 Running backend security tests..."
	cd servers/robot_controller_backend && \
		source venv/bin/activate && \
		pytest tests/security -v && \
		bandit -r . -f json -o security-report.json || true

test-performance: test-performance-frontend test-performance-backend
	@echo "✅ Performance tests completed"

test-performance-frontend:
	@echo "⚡ Running frontend performance tests..."
	cd ui/robot-controller-ui && npm run test:performance || echo "Performance tests not configured"

test-performance-backend: setup-venv
	@echo "⚡ Running backend performance tests..."
	cd servers/robot_controller_backend && \
		source venv/bin/activate && \
		pytest tests/performance -v

test-e2e:
	@echo "🌐 Running E2E tests..."
	cd ui/robot-controller-ui && npm run test:e2e:ci

# Linting and formatting
lint: lint-frontend lint-backend
	@echo "✅ Linting completed"

lint-frontend:
	@echo "🔍 Linting frontend..."
	cd ui/robot-controller-ui && npm run lint

lint-backend: setup-venv
	@echo "🔍 Linting backend..."
	cd servers/robot_controller_backend && \
		source venv/bin/activate && \
		flake8 . --max-line-length=120 --exclude=venv,__pycache__,.git || true && \
		mypy . --ignore-missing-imports || true

format: format-frontend format-backend
	@echo "✅ Formatting completed"

format-frontend:
	@echo "💅 Formatting frontend..."
	cd ui/robot-controller-ui && npm run format

format-backend: setup-venv
	@echo "💅 Formatting backend..."
	cd servers/robot_controller_backend && \
		source venv/bin/activate && \
		black . --line-length=120 --exclude=venv || true && \
		isort . --profile=black || true

# Installation
install: install-frontend install-backend
	@echo "✅ Installation completed"

install-frontend:
	@echo "📦 Installing frontend dependencies..."
	cd ui/robot-controller-ui && npm install

install-backend: setup-venv
	@echo "📦 Installing backend dependencies..."
	cd servers/robot_controller_backend && \
		source venv/bin/activate && \
		pip install -r requirements.txt

setup-venv:
	@echo "🐍 Setting up Python virtual environment..."
	@if [ ! -d "servers/robot-controller-backend/venv" ]; then \
		cd servers/robot_controller_backend && \
		python3 -m venv venv && \
		source venv/bin/activate && \
		pip install --upgrade pip && \
		pip install -r requirements.txt; \
	fi

# Cleanup
clean:
	@echo "🧹 Cleaning build artifacts..."
	cd ui/robot-controller-ui && rm -rf .next node_modules/.cache coverage
	cd servers/robot-controller-backend && rm -rf __pycache__ .pytest_cache htmlcov .coverage *.pyc
	find . -type d -name __pycache__ -exec rm -r {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
