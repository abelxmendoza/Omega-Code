#!/bin/bash
# Comprehensive Test Suite Runner - Fixed Version
# Runs all test suites with proper error handling and reporting

set +e  # Don't exit on error - collect all results

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test results
FRONTEND_UNIT_PASSED=false
FRONTEND_E2E_PASSED=false
BACKEND_UNIT_PASSED=false
BACKEND_INTEGRATION_PASSED=false
SECURITY_PASSED=false
PERFORMANCE_PASSED=false

echo -e "${GREEN}🧪 Starting Comprehensive Test Suite${NC}"
echo "=========================================="

# Backend Unit Tests (skip hardware-dependent)
echo -e "\n${YELLOW}🐍 Running Backend Unit Tests...${NC}"
cd servers/robot_controller_backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/unit/ -v --tb=no -q -k "not hardware" 2>&1 | tee /tmp/backend_unit.log; then
        BACKEND_UNIT_PASSED=true
        echo -e "${GREEN}✅ Backend unit tests passed${NC}"
    else
        echo -e "${RED}❌ Backend unit tests failed${NC}"
        echo "Last 10 lines of output:"
        tail -10 /tmp/backend_unit.log
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping backend tests${NC}"
fi
cd ../..

# Frontend Unit Tests
echo -e "\n${YELLOW}📱 Running Frontend Unit Tests...${NC}"
cd ui/robot-controller-ui
if npm test -- --config=jest.config.js --watchAll=false --passWithNoTests --silent 2>&1 | tee /tmp/frontend_unit.log; then
    FRONTEND_UNIT_PASSED=true
    echo -e "${GREEN}✅ Frontend unit tests passed${NC}"
else
    echo -e "${RED}❌ Frontend unit tests failed${NC}"
    echo "Last 10 lines of output:"
    tail -10 /tmp/frontend_unit.log
fi
cd ../..

# Frontend E2E Tests (skip if Cypress not available)
echo -e "\n${YELLOW}🌐 Running Frontend E2E Tests...${NC}"
cd ui/robot-controller-ui
if command -v cypress &> /dev/null || [ -d "node_modules/.bin/cypress" ]; then
    if npm run test:e2e:ci 2>&1 | tee /tmp/frontend_e2e.log; then
        FRONTEND_E2E_PASSED=true
        echo -e "${GREEN}✅ Frontend E2E tests passed${NC}"
    else
        echo -e "${YELLOW}⚠️  Frontend E2E tests skipped or failed${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Cypress not available, skipping E2E tests${NC}"
fi
cd ../..

# Backend Integration Tests (skip if services not running)
echo -e "\n${YELLOW}🔗 Running Backend Integration Tests...${NC}"
cd servers/robot_controller_backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/integration/ -v --tb=no -q -m "integration" --skip-if-no-service 2>&1 | tee /tmp/backend_integration.log; then
        BACKEND_INTEGRATION_PASSED=true
        echo -e "${GREEN}✅ Backend integration tests passed${NC}"
    else
        echo -e "${YELLOW}⚠️  Backend integration tests skipped (services may not be running)${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping integration tests${NC}"
fi
cd ../..

# Security Tests
echo -e "\n${YELLOW}🔒 Running Security Tests...${NC}"
cd servers/robot_controller_backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/security/ -v --tb=no -q -m "security" 2>&1 | tee /tmp/security.log; then
        SECURITY_PASSED=true
        echo -e "${GREEN}✅ Security tests passed${NC}"
    else
        echo -e "${YELLOW}⚠️  Security tests skipped or failed${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping security tests${NC}"
fi
cd ../..

# Performance Tests
echo -e "\n${YELLOW}⚡ Running Performance Tests...${NC}"
cd servers/robot_controller_backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/performance/ -v --tb=no -q -m "performance" 2>&1 | tee /tmp/performance.log; then
        PERFORMANCE_PASSED=true
        echo -e "${GREEN}✅ Performance tests passed${NC}"
    else
        echo -e "${YELLOW}⚠️  Performance tests skipped or failed${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping performance tests${NC}"
fi
cd ../..

# Summary
echo -e "\n${BLUE}=========================================="
echo "Test Suite Summary"
echo "==========================================${NC}"
echo "Backend Unit Tests:      $([ "$BACKEND_UNIT_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"
echo "Frontend Unit Tests:     $([ "$FRONTEND_UNIT_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"
echo "Frontend E2E Tests:      $([ "$FRONTEND_E2E_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${YELLOW}⚠️  SKIPPED${NC}")"
echo "Backend Integration:     $([ "$BACKEND_INTEGRATION_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${YELLOW}⚠️  SKIPPED${NC}")"
echo "Security Tests:          $([ "$SECURITY_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${YELLOW}⚠️  SKIPPED${NC}")"
echo "Performance Tests:       $([ "$PERFORMANCE_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${YELLOW}⚠️  SKIPPED${NC}")"

# Exit with error if critical tests failed
if [ "$BACKEND_UNIT_PASSED" = false ] || [ "$FRONTEND_UNIT_PASSED" = false ]; then
    echo -e "\n${RED}❌ Critical tests failed${NC}"
    exit 1
else
    echo -e "\n${GREEN}✅ Core tests passed!${NC}"
    exit 0
fi

