# Multi-stage Dockerfile for Robot Controller Backend
# Supports both Python and Go components

# Stage 1: Base image with common dependencies
FROM python:3.11-slim as base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    i2c-tools \
    libi2c-dev \
    build-essential \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy requirements first for better caching
COPY servers/robot-controller-backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Stage 2: Python backend
FROM base as python-backend

# Copy Python source code
COPY servers/robot-controller-backend/ ./

# Create non-root user for security
RUN useradd -m -u 1000 robotuser && \
    chown -R robotuser:robotuser /app

USER robotuser

# Expose ports
EXPOSE 8080 8081 8090

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8081/health || exit 1

# Default command
CMD ["python", "main_api.py"]

# Stage 3: Go backend (if needed separately)
FROM golang:1.21-alpine as go-backend

# Install system dependencies
RUN apk add --no-cache \
    git \
    ca-certificates \
    tzdata

WORKDIR /app

# Copy Go modules
COPY servers/robot-controller-backend/sensors/go.mod .
COPY servers/robot-controller-backend/sensors/go.sum .

# Download dependencies
RUN go mod download

# Copy Go source code
COPY servers/robot-controller-backend/sensors/ ./

# Build the application
RUN CGO_ENABLED=0 GOOS=linux go build -a -installsuffix cgo -o main .

# Create non-root user
RUN adduser -D -s /bin/sh robotuser

USER robotuser

EXPOSE 8082

CMD ["./main"]

# Stage 4: Production image
FROM python-backend as production

# Add production-specific configurations
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV ENVIRONMENT=production

# Copy production configuration
COPY servers/robot-controller-backend/config/ ./config/

# Add startup script
COPY servers/robot-controller-backend/scripts/start.sh ./start.sh
RUN chmod +x ./start.sh

CMD ["./start.sh"]

# Stage 5: Development image
FROM base as development

# Install development dependencies
RUN pip install --no-cache-dir \
    pytest \
    pytest-cov \
    pytest-asyncio \
    black \
    flake8 \
    mypy \
    bandit

# Copy source code
COPY servers/robot-controller-backend/ ./

# Create non-root user
RUN useradd -m -u 1000 robotuser && \
    chown -R robotuser:robotuser /app

USER robotuser

EXPOSE 8080 8081 8090

CMD ["python", "-m", "pytest", "tests/", "-v"]