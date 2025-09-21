#!/bin/bash

# Robot Controller Deployment Script
set -e

echo "🚀 Starting Robot Controller Deployment..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker-compose &> /dev/null; then
    print_error "Docker Compose is not installed. Please install Docker Compose first."
    exit 1
fi

# Create necessary directories
print_status "Creating necessary directories..."
mkdir -p logs
mkdir -p ssl
mkdir -p data

# Build and start services
print_status "Building and starting services..."
docker-compose down --remove-orphans
docker-compose build --no-cache
docker-compose up -d

# Wait for services to be ready
print_status "Waiting for services to be ready..."
sleep 10

# Check service health
print_status "Checking service health..."

# Check frontend
if curl -f http://localhost:3000 > /dev/null 2>&1; then
    print_success "Frontend is running on http://localhost:3000"
else
    print_warning "Frontend may not be ready yet"
fi

# Check backend
if curl -f http://localhost:8080/health > /dev/null 2>&1; then
    print_success "Backend is running on http://localhost:8080"
else
    print_warning "Backend may not be ready yet"
fi

# Check Redis
if docker-compose exec redis redis-cli ping > /dev/null 2>&1; then
    print_success "Redis is running"
else
    print_warning "Redis may not be ready yet"
fi

# Show running containers
print_status "Running containers:"
docker-compose ps

# Show logs
print_status "Recent logs:"
docker-compose logs --tail=20

print_success "Deployment completed!"
print_status "Access the application at:"
echo "  Frontend: http://localhost:3000"
echo "  Backend:  http://localhost:8080"
echo "  Analytics: http://localhost:3000/analytics"
echo "  Demo:     http://localhost:3000/demo"

print_status "To view logs: docker-compose logs -f"
print_status "To stop services: docker-compose down"
print_status "To restart services: docker-compose restart"
