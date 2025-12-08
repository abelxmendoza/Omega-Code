# 🔐 Security Best Practices - Omega-Code

## Overview

This document outlines security best practices implemented across the Omega-Code codebase while maintaining full accessibility for legitimate users.

---

## ✅ **Security Features Implemented**

### **1. API Security**

#### **CORS Protection**
- ✅ Configurable origin allowlist (no wildcard `*`)
- ✅ Reads from `config.yaml` → `security.allowed_origins`
- ✅ Falls back to environment variable `ALLOWED_ORIGINS`
- ✅ Defaults to localhost for development

#### **Rate Limiting**
- ✅ Prevents abuse and DoS attacks
- ✅ Configurable requests per minute (default: 60)
- ✅ Per-IP tracking
- ✅ Health checks excluded
- ✅ Automatic cleanup of old entries

#### **API Key Authentication** (Optional)
- ✅ Disabled by default (maintains accessibility)
- ✅ Public endpoints remain accessible:
  - `/health`
  - `/api/health`
  - `/api/capabilities`
  - `/api/system/mode/list`
  - `/api/system/mode/status`
- ✅ API key via `X-API-Key` header or `Authorization: Bearer <key>`

#### **CSRF Protection** (Optional)
- ✅ Enabled via `security.csrf_enabled: true`
- ✅ GET/HEAD/OPTIONS exempt (safe methods)
- ✅ Health checks exempt
- ✅ Requires `X-CSRF-Token` header for state-changing requests

#### **Request Size Limits**
- ✅ Maximum request body: 10MB (configurable)
- ✅ Maximum JSON depth: 20 levels
- ✅ Prevents JSON bomb attacks

---

### **2. WebSocket Security**

#### **Origin Validation**
- ✅ Validates WebSocket origin header
- ✅ Configurable allowlist via `ORIGIN_ALLOW` env var
- ✅ Option to allow requests without origin (CLI tools)

#### **Rate Limiting**
- ✅ Maximum 100 messages per minute per connection
- ✅ Per-connection tracking
- ✅ Automatic cleanup

#### **Message Size Limits**
- ✅ Maximum message size: 1MB
- ✅ Prevents DoS via large messages

#### **Input Sanitization**
- ✅ Removes null bytes and control characters
- ✅ Validates JSON depth
- ✅ Sanitizes string values
- ✅ Prevents injection attacks

#### **Connection Limits**
- ✅ Maximum 10 concurrent WebSocket connections
- ✅ Prevents resource exhaustion

---

### **3. Input Validation**

#### **String Validation**
- ✅ Removes null bytes
- ✅ Removes control characters (except newline, tab, carriage return)
- ✅ Length limits (configurable per field)
- ✅ Character set validation (for specific fields)

#### **Path Validation**
- ✅ Prevents path traversal (`../`)
- ✅ Removes dangerous characters (`<`, `>`, `|`, `&`, etc.)
- ✅ Validates file paths

#### **JSON Validation**
- ✅ Depth limits (prevents JSON bombs)
- ✅ Size limits
- ✅ Structure validation

#### **Type Validation**
- ✅ Strict type checking
- ✅ Range validation (ports, speeds, brightness, etc.)
- ✅ Format validation (IP addresses, hex colors, etc.)

---

### **4. Secret Management**

#### **Password Handling**
- ✅ Passwords stored in `config.yaml` (not committed to git)
- ✅ Never logged in plain text
- ✅ Masked in all log messages
- ✅ Environment variable fallback

#### **API Key Handling**
- ✅ Stored in `config.yaml` or environment variables
- ✅ Never logged
- ✅ Masked in error messages
- ✅ Optional authentication (disabled by default)

#### **Secret Masking**
- ✅ Automatic masking in logs
- ✅ Pattern matching for common secret fields
- ✅ Sanitization before logging

---

### **5. Security Headers**

#### **Implemented Headers**
- ✅ `X-Content-Type-Options: nosniff` - Prevents MIME sniffing
- ✅ `X-Frame-Options: DENY` - Prevents clickjacking
- ✅ `X-XSS-Protection: 1; mode=block` - XSS protection
- ✅ `Referrer-Policy: strict-origin-when-cross-origin` - Controls referrer
- ✅ `Content-Security-Policy` - XSS and injection protection
- ✅ `Permissions-Policy` - Feature access control
- ✅ `Strict-Transport-Security` - HSTS (HTTPS only)
- ✅ Server header removed - Prevents info leakage

---

### **6. Security Audit Logging**

#### **Logged Events**
- ✅ Failed authentication attempts (401)
- ✅ Forbidden access attempts (403)
- ✅ Rate limit violations (429)
- ✅ Slow requests (>5 seconds)
- ✅ WebSocket origin rejections
- ✅ Invalid input attempts
- ✅ Security exceptions

#### **Log Format**
- ✅ Timestamped
- ✅ IP address logged
- ✅ Path and method logged
- ✅ Secrets masked
- ✅ Structured format for analysis

---

### **7. Error Handling**

#### **Secure Error Messages**
- ✅ Generic error messages to clients
- ✅ Detailed errors logged server-side only
- ✅ No stack traces exposed
- ✅ No sensitive information leaked
- ✅ User-friendly error messages (maintains accessibility)

---

## 🔧 **Configuration**

### **config.yaml**

```yaml
security:
  api_auth_enabled: false  # Set to true to enable API key auth
  api_key: ''  # Set in environment variable or here (not committed)
  allowed_origins:
    - http://localhost:3000
    - http://omega1.local:3000
    - https://your-vercel-app.vercel.app
  rate_limit_enabled: true
  requests_per_minute: 60
  csrf_enabled: false  # Set to true to enable CSRF protection
  request_size_limit_mb: 10
  audit_logging: true
```

### **Environment Variables**

```bash
# API Security
API_KEY=your-secret-key-here
API_AUTH_ENABLED=false
ALLOWED_ORIGINS=http://localhost:3000,http://omega1.local:3000

# Rate Limiting
REQUESTS_PER_MINUTE=60

# CSRF Protection
CSRF_ENABLED=false

# Request Limits
REQUEST_SIZE_LIMIT_MB=10

# Audit Logging
AUDIT_LOGGING=true

# WebSocket Security
ORIGIN_ALLOW=http://localhost:3000,https://your-app.vercel.app
ORIGIN_ALLOW_NO_HEADER=0  # Set to 1 to allow CLI tools
```

---

## 🛡️ **Security Checklist**

### **Before Deployment**

- [ ] Set `api_auth_enabled: true` if deploying publicly
- [ ] Generate strong API key (32+ characters, random)
- [ ] Configure `allowed_origins` with your actual domains
- [ ] Set `csrf_enabled: true` for production
- [ ] Review `request_size_limit_mb` (default 10MB)
- [ ] Ensure `.env` file is in `.gitignore`
- [ ] Ensure `config.yaml` doesn't contain real passwords in git
- [ ] Enable HTTPS/TLS in production
- [ ] Review security audit logs regularly
- [ ] Set up log rotation for security logs

### **Development**

- [ ] Use `localhost` origins for local development
- [ ] Keep `api_auth_enabled: false` for easy testing
- [ ] Use strong passwords even in dev (don't commit them)
- [ ] Review security logs for suspicious activity
- [ ] Test rate limiting doesn't break legitimate use
- [ ] Test CSRF protection (if enabled)

---

## 🚨 **Security Incident Response**

### **If You Suspect a Breach**

1. **Immediately**:
   - Enable API authentication (`api_auth_enabled: true`)
   - Rotate API keys
   - Review security audit logs
   - Check for unauthorized access patterns

2. **Investigate**:
   - Check `/var/log/omega/security_audit.log`
   - Review rate limit violations
   - Check for failed authentication attempts
   - Review WebSocket connection logs

3. **Remediate**:
   - Update passwords/API keys
   - Tighten CORS origins
   - Adjust rate limits if needed
   - Update security configuration

---

## 📊 **Security Monitoring**

### **What to Monitor**

1. **Rate Limit Violations**
   - Sudden spikes may indicate attack
   - Review IP addresses

2. **Failed Authentication**
   - Multiple 401 errors from same IP
   - May indicate brute force attempt

3. **Forbidden Access**
   - 403 errors may indicate unauthorized access attempts
   - Review paths being accessed

4. **Slow Requests**
   - Requests >5 seconds may indicate DoS attempt
   - Review request patterns

5. **WebSocket Rejections**
   - Origin rejections may indicate CSRF attempts
   - Review origin patterns

---

## ✅ **Accessibility Maintained**

All security features are designed to **not interfere** with legitimate use:

- ✅ Public endpoints remain accessible (health checks, capabilities)
- ✅ Rate limits are generous (60 req/min default)
- ✅ CSRF protection can be disabled for API-only use
- ✅ Authentication is optional (disabled by default)
- ✅ Error messages are user-friendly
- ✅ No breaking changes to existing functionality

---

## 🔗 **Related Files**

- `api/security_middleware.py` - Core security middleware
- `api/security_enhancements.py` - Enhanced security features
- `api/websocket_security.py` - WebSocket security
- `api/input_validators.py` - Input validation utilities
- `api/error_handlers.py` - Secure error handling
- `omega_config/config.yaml` - Security configuration
- `tests/security/` - Security tests

---

## 📚 **References**

- [OWASP Top 10](https://owasp.org/www-project-top-ten/)
- [FastAPI Security](https://fastapi.tiangolo.com/tutorial/security/)
- [WebSocket Security](https://datatracker.ietf.org/doc/html/rfc6455#section-10)
- [CORS Best Practices](https://developer.mozilla.org/en-US/docs/Web/HTTP/CORS)

---

## 🎯 **Summary**

**Security Features**:
- ✅ CORS protection
- ✅ Rate limiting
- ✅ Optional API key auth
- ✅ Optional CSRF protection
- ✅ Request size limits
- ✅ Enhanced security headers
- ✅ WebSocket security
- ✅ Input validation
- ✅ Secret masking
- ✅ Security audit logging

**Accessibility Maintained**:
- ✅ Public endpoints accessible
- ✅ Optional authentication
- ✅ User-friendly error messages
- ✅ No breaking changes
- ✅ Configurable security levels

**Your robot is secure AND accessible!** 🔐✨

