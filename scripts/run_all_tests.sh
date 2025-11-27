#!/bin/bash
# Comprehensive Test Suite Runner
# Runs all test suites with proper error handling and reporting

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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

# Frontend Unit Tests
echo -e "\n${YELLOW}📱 Running Frontend Unit Tests...${NC}"
cd ui/robot-controller-ui
if npm test -- --watchAll=false --coverage; then
    FRONTEND_UNIT_PASSED=true
    echo -e "${GREEN}✅ Frontend unit tests passed${NC}"
else
    echo -e "${RED}❌ Frontend unit tests failed${NC}"
fi
cd ../..

# Frontend E2E Tests
echo -e "\n${YELLOW}🌐 Running Frontend E2E Tests...${NC}"
cd ui/robot-controller-ui
if npm run test:e2e:ci; then
    FRONTEND_E2E_PASSED=true
    echo -e "${GREEN}✅ Frontend E2E tests passed${NC}"
else
    echo -e "${RED}❌ Frontend E2E tests failed${NC}"
fi
cd ../..

# Backend Unit Tests
echo -e "\n${YELLOW}🐍 Running Backend Unit Tests...${NC}"
cd servers/robot-controller-backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/unit -v; then
        BACKEND_UNIT_PASSED=true
        echo -e "${GREEN}✅ Backend unit tests passed${NC}"
    else
        echo -e "${RED}❌ Backend unit tests failed${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping backend tests${NC}"
fi
cd ../..

# Backend Integration Tests
echo -e "\n${YELLOW}🔗 Running Backend Integration Tests...${NC}"
cd servers/robot-controller-backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/integration -v -m integration; then
        BACKEND_INTEGRATION_PASSED=true
        echo -e "${GREEN}✅ Backend integration tests passed${NC}"
    else
        echo -e "${RED}❌ Backend integration tests failed${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping backend tests${NC}"
fi
cd ../..

# Security Tests
echo -e "\n${YELLOW}🔒 Running Security Tests...${NC}"
cd servers/robot-controller-backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/security -v -m security; then
        SECURITY_PASSED=true
        echo -e "${GREEN}✅ Security tests passed${NC}"
    else
        echo -e "${RED}❌ Security tests failed${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping security tests${NC}"
fi
cd ../..

# Performance Tests
echo -e "\n${YELLOW}⚡ Running Performance Tests...${NC}"
cd servers/robot-controller-backend
if [ -d "venv" ]; then
    source venv/bin/activate
    if pytest tests/performance -v -m performance; then
        PERFORMANCE_PASSED=true
        echo -e "${GREEN}✅ Performance tests passed${NC}"
    else
        echo -e "${RED}❌ Performance tests failed${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  Virtual environment not found, skipping performance tests${NC}"
fi
cd ../..

# Summary
echo -e "\n${GREEN}=========================================="
echo "Test Suite Summary"
echo "==========================================${NC}"
echo "Frontend Unit Tests:     $([ "$FRONTEND_UNIT_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"
echo "Frontend E2E Tests:      $([ "$FRONTEND_E2E_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"
echo "Backend Unit Tests:      $([ "$BACKEND_UNIT_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"
echo "Backend Integration:     $([ "$BACKEND_INTEGRATION_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"
echo "Security Tests:          $([ "$SECURITY_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"
echo "Performance Tests:       $([ "$PERFORMANCE_PASSED" = true ] && echo -e "${GREEN}✅ PASSED${NC}" || echo -e "${RED}❌ FAILED${NC}")"

# Exit with error if any tests failed
if [ "$FRONTEND_UNIT_PASSED" = false ] || \
   [ "$FRONTEND_E2E_PASSED" = false ] || \
   [ "$BACKEND_UNIT_PASSED" = false ] || \
   [ "$BACKEND_INTEGRATION_PASSED" = false ] || \
   [ "$SECURITY_PASSED" = false ] || \
   [ "$PERFORMANCE_PASSED" = false ]; then
    echo -e "\n${RED}❌ Some tests failed${NC}"
    exit 1
else
    echo -e "\n${GREEN}✅ All tests passed!${NC}"
    exit 0
fi

