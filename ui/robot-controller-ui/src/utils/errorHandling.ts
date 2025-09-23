/*
# File: /src/utils/errorHandling.ts
# Summary:
# Comprehensive error handling utilities for the frontend
# - Error logging and reporting
# - User-friendly error messages
# - Error recovery strategies
# - Error monitoring integration
*/

export interface ErrorInfo {
  message: string;
  stack?: string;
  component?: string;
  timestamp: number;
  userId?: string;
  sessionId?: string;
  url?: string;
  userAgent?: string;
}

export interface ErrorContext {
  component: string;
  action?: string;
  data?: any;
  userId?: string;
}

export class ErrorHandler {
  private static instance: ErrorHandler;
  private errorCount = 0;
  private maxErrors = 100;
  private sessionId: string;

  private constructor() {
    this.sessionId = this.generateSessionId();
  }

  public static getInstance(): ErrorHandler {
    if (!ErrorHandler.instance) {
      ErrorHandler.instance = new ErrorHandler();
    }
    return ErrorHandler.instance;
  }

  private generateSessionId(): string {
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public handleError(error: Error | string, context?: ErrorContext): void {
    this.errorCount++;
    
    if (this.errorCount > this.maxErrors) {
      console.error('Too many errors, stopping error reporting');
      return;
    }

    const errorInfo: ErrorInfo = {
      message: typeof error === 'string' ? error : error.message,
      stack: typeof error === 'string' ? undefined : error.stack,
      component: context?.component || 'Unknown',
      timestamp: Date.now(),
      sessionId: this.sessionId,
      url: window.location.href,
      userAgent: navigator.userAgent,
    };

    // Log to console
    console.error('ErrorHandler:', errorInfo);

    // Send to monitoring service (if available)
    this.reportError(errorInfo);

    // Show user notification for critical errors
    if (this.isCriticalError(error, context)) {
      this.showUserNotification(errorInfo);
    }
  }

  private isCriticalError(error: Error | string, context?: ErrorContext): boolean {
    const message = typeof error === 'string' ? error : error.message;
    
    // Critical error patterns
    const criticalPatterns = [
      'WebSocket connection failed',
      'Failed to initialize',
      'Hardware communication error',
      'Motor control error',
      'Servo control error',
    ];

    return criticalPatterns.some(pattern => 
      message.toLowerCase().includes(pattern.toLowerCase())
    );
  }

  private showUserNotification(errorInfo: ErrorInfo): void {
    // Create a user-friendly notification
    const notification = document.createElement('div');
    notification.className = 'error-notification';
    notification.innerHTML = `
      <div class="error-notification-content">
        <div class="error-icon">⚠️</div>
        <div class="error-message">
          <strong>Connection Issue</strong>
          <p>There's a problem with the robot connection. Please check your network and try refreshing the page.</p>
          <button onclick="this.parentElement.parentElement.remove()">Dismiss</button>
        </div>
      </div>
    `;

    // Add styles
    notification.style.cssText = `
      position: fixed;
      top: 20px;
      right: 20px;
      background: #ff4444;
      color: white;
      padding: 15px;
      border-radius: 8px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.3);
      z-index: 10000;
      max-width: 400px;
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    `;

    document.body.appendChild(notification);

    // Auto-remove after 10 seconds
    setTimeout(() => {
      if (notification.parentElement) {
        notification.remove();
      }
    }, 10000);
  }

  private reportError(errorInfo: ErrorInfo): void {
    // In a real application, you would send this to your error monitoring service
    try {
      // Example: Send to monitoring service
      // fetch('/api/errors', {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify(errorInfo),
      // });

      // For now, just log it
      console.log('Error report prepared:', errorInfo);
    } catch (reportError) {
      console.error('Failed to report error:', reportError);
    }
  }

  public getErrorCount(): number {
    return this.errorCount;
  }

  public resetErrorCount(): void {
    this.errorCount = 0;
  }
}

// Convenience functions
export const handleError = (error: Error | string, context?: ErrorContext): void => {
  ErrorHandler.getInstance().handleError(error, context);
};

export const logError = (error: Error | string, context?: ErrorContext): void => {
  console.error(`[${context?.component || 'Unknown'}]`, error);
  ErrorHandler.getInstance().handleError(error, context);
};

// WebSocket error handling
export const handleWebSocketError = (error: Event | Error, context?: ErrorContext): void => {
  const errorMessage = error instanceof Error ? error.message : 'WebSocket connection error';
  handleError(errorMessage, { ...context, action: 'websocket' } as ErrorContext);
};

// API error handling
export const handleApiError = (error: Error, endpoint: string, context?: ErrorContext): void => {
  handleError(`API Error (${endpoint}): ${error.message}`, { ...context, action: 'api' } as ErrorContext);
};

// Component error handling
export const handleComponentError = (error: Error, componentName: string, action?: string): void => {
  handleError(error, { component: componentName, action });
};

// Hardware error handling
export const handleHardwareError = (error: Error, device: string, context?: ErrorContext): void => {
  handleError(`Hardware Error (${device}): ${error.message}`, { ...context, action: 'hardware' } as ErrorContext);
};

// Error boundary helper
export const createErrorBoundaryFallback = (error: Error, errorInfo: any) => {
  return {
    error,
    errorInfo,
    timestamp: Date.now(),
    sessionId: ErrorHandler.getInstance()['sessionId'],
  };
};

// Retry mechanism
export const withRetry = async <T>(
  fn: () => Promise<T>,
  maxRetries: number = 3,
  delay: number = 1000,
  context?: ErrorContext
): Promise<T> => {
  let lastError: Error;
  
  for (let attempt = 1; attempt <= maxRetries; attempt++) {
    try {
      return await fn();
    } catch (error) {
      lastError = error as Error;
      
      if (attempt === maxRetries) {
        handleError(`Retry failed after ${maxRetries} attempts: ${lastError.message}`, context);
        throw lastError;
      }
      
      console.warn(`Attempt ${attempt} failed, retrying in ${delay}ms:`, error);
      await new Promise(resolve => setTimeout(resolve, delay));
      delay *= 2; // Exponential backoff
    }
  }
  
  throw lastError!;
};

// Safe async wrapper
export const safeAsync = async <T>(
  fn: () => Promise<T>,
  fallback: T,
  context?: ErrorContext
): Promise<T> => {
  try {
    return await fn();
  } catch (error) {
    handleError(error as Error, context);
    return fallback;
  }
};

// Safe sync wrapper
export const safeSync = <T>(
  fn: () => T,
  fallback: T,
  context?: ErrorContext
): T => {
  try {
    return fn();
  } catch (error) {
    handleError(error as Error, context);
    return fallback;
  }
};

export default ErrorHandler;
