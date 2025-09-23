# Robot Controller CI/CD Pipeline

This document describes the comprehensive CI/CD pipeline for the Robot Controller application, including both frontend and backend components.

## Overview

The CI/CD pipeline includes:
- **Frontend**: Next.js React application with automated testing, building, and deployment to Vercel
- **Backend**: Python/Go services with Docker containerization and automated deployment
- **Testing**: Unit tests, integration tests, performance tests, and E2E tests
- **Security**: Automated security scanning and vulnerability assessment
- **Monitoring**: Performance monitoring and error tracking
- **Deployment**: Automated deployment to staging and production environments

## Pipeline Structure

### Frontend Pipeline (`frontend-ci-cd.yml`)

#### Triggers
- Push to `master`, `main`, or `develop` branches
- Pull requests to `master` or `main`
- Manual workflow dispatch

#### Jobs

1. **Lint & Type Check**
   - ESLint code quality checks
   - TypeScript type checking
   - Dependency analysis

2. **Unit Tests**
   - Jest test execution
   - Coverage reporting
   - Code coverage upload to Codecov

3. **Build Application**
   - Next.js production build
   - Bundle size analysis
   - Build artifact upload

4. **Security Scan**
   - npm audit for vulnerabilities
   - Snyk security scanning
   - Dependency vulnerability checks

5. **E2E Tests**
   - Cypress end-to-end testing
   - Screenshot and video capture on failures
   - Cross-browser testing

6. **Performance Tests**
   - Lighthouse CI performance testing
   - Core Web Vitals measurement
   - Performance budget enforcement

7. **Deploy to Production**
   - Automatic deployment to Vercel production
   - Environment variable configuration
   - Health checks

8. **Deploy Preview**
   - Preview deployment for pull requests
   - Automatic PR comments with preview URLs

### Backend Pipeline (`backend-ci-cd.yml`)

#### Triggers
- Push to `master`, `main`, or `develop` branches
- Pull requests to `master` or `main`
- Manual workflow dispatch

#### Jobs

1. **Python Tests**
   - Multi-version Python testing (3.9, 3.10, 3.11)
   - pytest execution with coverage
   - System dependency installation

2. **Go Tests**
   - Multi-version Go testing (1.20, 1.21, 1.22)
   - Race condition detection
   - Coverage reporting

3. **Lint & Code Quality**
   - Python: Black, flake8, isort, mypy, bandit
   - Go: golangci-lint
   - Code formatting and style checks

4. **Security Scan**
   - Python: safety, semgrep
   - Dependency vulnerability scanning
   - Security report generation

5. **Build Docker Images**
   - Multi-architecture Docker builds
   - Docker Hub image publishing
   - Build cache optimization

6. **Integration Tests**
   - Full system integration testing
   - WebSocket communication tests
   - Service interaction validation

7. **Performance Tests**
   - Load testing with Locust
   - Benchmark testing
   - Performance regression detection

8. **Deploy to Staging**
   - Automatic deployment to staging environment
   - Docker Compose deployment
   - Health checks

9. **Deploy to Production**
   - Production deployment with approval gates
   - Zero-downtime deployment
   - Rollback capabilities

## Configuration

### Required Secrets

#### Frontend Secrets
- `VERCEL_TOKEN`: Vercel deployment token
- `VERCEL_ORG_ID`: Vercel organization ID
- `VERCEL_PROJECT_ID`: Vercel project ID
- `SNYK_TOKEN`: Snyk security scanning token
- `SLACK_WEBHOOK`: Slack notification webhook

#### Backend Secrets
- `DOCKER_USERNAME`: Docker Hub username
- `DOCKER_PASSWORD`: Docker Hub password
- `STAGING_HOST`: Staging server hostname
- `STAGING_USERNAME`: Staging server username
- `STAGING_SSH_KEY`: Staging server SSH key
- `PRODUCTION_HOST`: Production server hostname
- `PRODUCTION_USERNAME`: Production server username
- `PRODUCTION_SSH_KEY`: Production server SSH key

### Environment Variables

#### Frontend
```bash
NEXT_PUBLIC_NETWORK_PROFILE=production
NEXT_PUBLIC_BACKEND_WS_URL_PROD=ws://your-domain.com:8081/
NEXT_PUBLIC_VIDEO_STREAM_URL_PROD=http://your-domain.com:8080/video_feed
```

#### Backend
```bash
ENVIRONMENT=production
LOG_LEVEL=INFO
PYTHONUNBUFFERED=1
```

## Testing Strategy

### Frontend Testing
- **Unit Tests**: Component and utility function testing
- **Integration Tests**: API integration and state management
- **E2E Tests**: Full user journey testing
- **Performance Tests**: Lighthouse CI performance metrics
- **Accessibility Tests**: WCAG compliance testing

### Backend Testing
- **Unit Tests**: Individual function and class testing
- **Integration Tests**: Service interaction testing
- **Performance Tests**: Load and stress testing
- **Security Tests**: Vulnerability and penetration testing
- **Hardware Tests**: Mock hardware interaction testing

## Deployment Strategy

### Frontend Deployment
1. **Preview Deployments**: Automatic for all pull requests
2. **Staging Deployment**: Automatic for `develop` branch
3. **Production Deployment**: Automatic for `master`/`main` branch

### Backend Deployment
1. **Staging Deployment**: Automatic for `develop` branch
2. **Production Deployment**: Automatic for `master`/`main` branch with health checks

## Monitoring and Observability

### Metrics Collection
- **Prometheus**: Metrics collection and storage
- **Grafana**: Metrics visualization and dashboards
- **ELK Stack**: Log aggregation and analysis

### Health Checks
- **Application Health**: HTTP health check endpoints
- **Service Health**: WebSocket connection monitoring
- **Hardware Health**: Hardware component status

### Alerting
- **Error Alerts**: Critical error notifications
- **Performance Alerts**: Performance degradation alerts
- **Security Alerts**: Security incident notifications

## Security

### Automated Security Scanning
- **Dependency Scanning**: npm audit, safety checks
- **Code Scanning**: Semgrep, bandit
- **Container Scanning**: Docker image vulnerability scanning
- **Secrets Scanning**: Secret detection in code

### Security Best Practices
- **Least Privilege**: Minimal required permissions
- **Secret Management**: Secure secret storage and rotation
- **Network Security**: Firewall and network isolation
- **Container Security**: Non-root containers, minimal base images

## Performance Optimization

### Frontend Optimization
- **Bundle Analysis**: Webpack bundle analyzer
- **Code Splitting**: Dynamic imports and lazy loading
- **Image Optimization**: Next.js image optimization
- **Caching**: Static asset caching strategies

### Backend Optimization
- **Docker Optimization**: Multi-stage builds, layer caching
- **Resource Limits**: CPU and memory constraints
- **Connection Pooling**: Database and service connections
- **Caching**: Redis caching for frequently accessed data

## Troubleshooting

### Common Issues

#### Frontend Pipeline Failures
- **Build Failures**: Check TypeScript errors and dependencies
- **Test Failures**: Review test output and coverage requirements
- **Deployment Failures**: Verify Vercel configuration and secrets

#### Backend Pipeline Failures
- **Test Failures**: Check test environment and mock configurations
- **Docker Build Failures**: Verify Dockerfile and build context
- **Deployment Failures**: Check server connectivity and permissions

### Debugging Steps
1. Check GitHub Actions logs for detailed error messages
2. Verify all required secrets are configured
3. Test locally with the same environment variables
4. Check service health and connectivity
5. Review monitoring dashboards for system status

## Maintenance

### Regular Tasks
- **Dependency Updates**: Monthly dependency updates
- **Security Patches**: Immediate security patch application
- **Performance Reviews**: Quarterly performance analysis
- **Pipeline Optimization**: Continuous pipeline improvement

### Monitoring Tasks
- **Log Review**: Daily log analysis
- **Metric Analysis**: Weekly performance metric review
- **Alert Tuning**: Monthly alert threshold adjustment
- **Capacity Planning**: Quarterly capacity assessment

## Contributing

### Adding New Tests
1. Create test files in appropriate directories
2. Follow existing test patterns and conventions
3. Add test markers for categorization
4. Update CI/CD pipeline if needed

### Modifying Pipeline
1. Test changes in a feature branch
2. Update documentation
3. Review with team before merging
4. Monitor pipeline performance after changes

## Support

For issues with the CI/CD pipeline:
1. Check GitHub Actions logs
2. Review this documentation
3. Contact the development team
4. Create an issue in the repository

---

This CI/CD pipeline ensures reliable, secure, and performant deployment of the Robot Controller application while maintaining high code quality and system reliability.
