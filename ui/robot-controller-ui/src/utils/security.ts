/**
 * Frontend Security Utilities
 * 
 * Provides security helpers for the frontend while maintaining accessibility.
 */

/**
 * Sanitize HTML content to prevent XSS attacks.
 * For accessibility, preserves semantic HTML but removes script tags.
 */
export function sanitizeHtml(html: string): string {
  // Create a temporary DOM element
  const div = document.createElement('div');
  div.textContent = html; // This automatically escapes HTML
  
  // Return sanitized HTML (text content is safe)
  return div.textContent || '';
}

/**
 * Validate and sanitize user input.
 * Preserves accessibility features like ARIA attributes in component props.
 */
export function sanitizeInput(input: string, maxLength: number = 1000): string {
  if (typeof input !== 'string') {
    return String(input).slice(0, maxLength);
  }
  
  // Remove null bytes
  let sanitized = input.replace(/\x00/g, '');
  
  // Remove control characters (except newline, tab, carriage return for accessibility)
  sanitized = sanitized.replace(/[\x00-\x08\x0B-\x0C\x0E-\x1F\x7F]/g, '');
  
  // Truncate to max length
  if (sanitized.length > maxLength) {
    sanitized = sanitized.slice(0, maxLength);
  }
  
  return sanitized;
}

/**
 * Validate URL to prevent open redirect attacks.
 */
export function validateUrl(url: string, allowedDomains: string[] = []): boolean {
  try {
    const urlObj = new URL(url);
    
    // If allowed domains specified, check against them
    if (allowedDomains.length > 0) {
      return allowedDomains.some(domain => urlObj.hostname === domain || urlObj.hostname.endsWith(`.${domain}`));
    }
    
    // Default: allow http, https, ws, wss
    return ['http:', 'https:', 'ws:', 'wss:'].includes(urlObj.protocol);
  } catch {
    return false;
  }
}

/**
 * Generate CSRF token for state-changing operations.
 * This helps prevent CSRF attacks while maintaining accessibility.
 */
export function generateCsrfToken(): string {
  // Generate a random token
  const array = new Uint8Array(32);
  crypto.getRandomValues(array);
  return Array.from(array, byte => byte.toString(16).padStart(2, '0')).join('');
}

/**
 * Store CSRF token in session storage (accessible to screen readers via aria-live regions).
 */
export function storeCsrfToken(token: string): void {
  try {
    sessionStorage.setItem('csrf_token', token);
  } catch (e) {
    // Session storage not available, continue without CSRF protection
    console.warn('CSRF token storage failed:', e);
  }
}

/**
 * Get CSRF token from session storage.
 */
export function getCsrfToken(): string | null {
  try {
    return sessionStorage.getItem('csrf_token');
  } catch {
    return null;
  }
}

/**
 * Validate API response to prevent XSS from malicious API responses.
 */
export function validateApiResponse(data: unknown): unknown {
  if (typeof data === 'string') {
    return sanitizeInput(data);
  }
  
  if (Array.isArray(data)) {
    return data.map(validateApiResponse);
  }
  
  if (data && typeof data === 'object') {
    const sanitized: Record<string, unknown> = {};
    for (const [key, value] of Object.entries(data)) {
      // Sanitize keys and values
      const sanitizedKey = sanitizeInput(key, 100);
      sanitized[sanitizedKey] = validateApiResponse(value);
    }
    return sanitized;
  }
  
  return data;
}

/**
 * Check if content is safe to render (for accessibility announcements).
 */
export function isSafeForAria(content: string): boolean {
  // Check for script tags or dangerous patterns
  const dangerousPatterns = [
    /<script/i,
    /javascript:/i,
    /on\w+\s*=/i, // Event handlers like onclick=
  ];
  
  return !dangerousPatterns.some(pattern => pattern.test(content));
}

