# Security Guide — Omega-Code

## Configuration

### `config.yaml`

```yaml
security:
  api_auth_enabled: false      # Set true to require API key
  api_key: ''                  # Store in env var, not committed to git
  allowed_origins:
    - http://localhost:3000
    - http://omega1.local:3000
    - https://your-vercel-app.vercel.app
  rate_limit_enabled: true
  requests_per_minute: 60
  csrf_enabled: false          # Set true for production
  request_size_limit_mb: 10
  audit_logging: true
```

### Environment variables

```bash
# Backend
API_KEY=your-secret-key-here
API_AUTH_ENABLED=false
ALLOWED_ORIGINS=http://localhost:3000,http://omega1.local:3000
REQUESTS_PER_MINUTE=60
CSRF_ENABLED=false
REQUEST_SIZE_LIMIT_MB=10
AUDIT_LOGGING=true

# WebSocket
ORIGIN_ALLOW=http://localhost:3000,https://your-app.vercel.app
ORIGIN_ALLOW_NO_HEADER=0    # Set to 1 to allow CLI tools (wscat, etc.)

# Frontend
NEXT_PUBLIC_CSP_NONCE=your-nonce-here    # Optional
```

---

## Features Implemented

### API Security (`api/security_middleware.py`)

- **CORS**: Configurable origin allowlist — no wildcard `*`, reads from `config.yaml` or `ALLOWED_ORIGINS`
- **Rate limiting**: 60 req/min per IP, configurable, health checks excluded
- **API key auth**: Optional (`api_auth_enabled: false` by default). Public endpoints always accessible: `/health`, `/api/capabilities`, `/api/system/mode/*`
- **CSRF protection**: Optional, disabled by default. Requires `X-CSRF-Token` header on state-changing requests
- **Request size limits**: Max 10MB body, max JSON depth 20 levels (prevents JSON bomb attacks)
- **Security headers**: `X-Content-Type-Options`, `X-Frame-Options: DENY`, `X-XSS-Protection`, `Referrer-Policy`, `Content-Security-Policy`, `Strict-Transport-Security`, server header removed

### WebSocket Security

- **Origin validation**: Allowlist via `ORIGIN_ALLOW`
- **Rate limiting**: 100 messages/min per connection
- **Message size limit**: 1MB max
- **Input sanitization**: Removes null bytes, control characters, validates JSON depth
- **Connection limit**: Max 10 concurrent WebSocket connections

### Input Validation (`api/input_validators.py`)

Validators for: robot name, Wi-Fi SSID/password, IP address, port number, hex color, brightness, motor speed, JSON depth/size, file path traversal.

```python
from api.input_validators import validate_robot_name, sanitize_json_input

try:
    name = validate_robot_name(request.name)
    data = sanitize_json_input(request.data)
except ValueError as e:
    raise HTTPException(status_code=400, detail=str(e))
```

### Frontend Security (`ui/robot-controller-ui/src/utils/security.ts`)

```typescript
import { sanitizeInput, validateUrl, generateCsrfToken } from '@/utils/security';

const safeInput = sanitizeInput(userInput);
const isValid = validateUrl(url, ['omega1.local']);
const token = generateCsrfToken();
```

### Error Handling (`api/error_handlers.py`)

- Generic messages to clients, full details logged server-side only
- No stack traces exposed, no secrets in error output

### Audit Logging

Events logged: failed auth (401), forbidden access (403), rate limit violations (429), slow requests (>5s), WebSocket origin rejections, invalid input attempts.

Log file: `/var/log/omega/security_audit.log`

---

## Deployment Checklist

### Before deploying publicly

- [ ] Set `api_auth_enabled: true`
- [ ] Generate strong API key (32+ random characters)
- [ ] Configure `allowed_origins` with actual domains
- [ ] Set `csrf_enabled: true`
- [ ] Ensure `.env` and `config.yaml` are in `.gitignore`
- [ ] Enable HTTPS/TLS
- [ ] Set up log rotation for security logs

### Development

- [ ] Use `localhost` in allowed origins
- [ ] Keep `api_auth_enabled: false` for easy testing
- [ ] Never commit real passwords or API keys

---

## Security Incident Response

1. **Enable API auth immediately**: Set `api_auth_enabled: true`, rotate API keys
2. **Review logs**: Check `/var/log/omega/security_audit.log` for 401/403/429 patterns
3. **Tighten config**: Update `allowed_origins`, adjust rate limits, rotate credentials

---

## Related Files

| File | Purpose |
|------|---------|
| `api/security_middleware.py` | CORS, rate limiting, auth, headers |
| `api/security_enhancements.py` | Enhanced security features |
| `api/websocket_security.py` | WebSocket security |
| `api/input_validators.py` | Input validation utilities |
| `api/error_handlers.py` | Secure error handling |
| `omega_config/config.yaml` | Security configuration |
| `ui/src/utils/security.ts` | Frontend security utilities |
| `tests/security/` | Security tests |
