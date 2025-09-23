/*
# File: /src/components/ErrorBoundary.tsx
# Summary:
# Enhanced error boundary component with detailed error reporting and recovery options
# - Catches JavaScript errors in component tree
# - Displays user-friendly error messages
# - Provides error details for debugging
# - Offers recovery options (retry, reset, report)
# - Logs errors for monitoring
*/

import React, { Component, ErrorInfo, ReactNode } from 'react';

interface ErrorBoundaryState {
  hasError: boolean;
  error: Error | null;
  errorInfo: ErrorInfo | null;
  errorId: string;
  retryCount: number;
}

interface ErrorBoundaryProps {
  children: ReactNode;
  fallback?: ReactNode;
  onError?: (error: Error, errorInfo: ErrorInfo, errorId: string) => void;
  maxRetries?: number;
  resetOnPropsChange?: boolean;
}

class ErrorBoundary extends Component<ErrorBoundaryProps, ErrorBoundaryState> {
  private retryTimeoutId: NodeJS.Timeout | null = null;

  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
      errorInfo: null,
      errorId: '',
      retryCount: 0,
    };
  }

  static getDerivedStateFromError(error: Error): Partial<ErrorBoundaryState> {
    // Generate unique error ID for tracking
    const errorId = `error_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    
    return {
      hasError: true,
      error,
      errorId,
    };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    const { onError } = this.props;
    const { errorId } = this.state;

    // Log error details
    console.error('ErrorBoundary caught an error:', {
      error,
      errorInfo,
      errorId,
      timestamp: new Date().toISOString(),
      userAgent: navigator.userAgent,
      url: window.location.href,
    });

    // Update state with error info
    this.setState({
      errorInfo,
    });

    // Call custom error handler
    onError?.(error, errorInfo, errorId);

    // Send error to monitoring service (if available)
    this.reportError(error, errorInfo, errorId);
  }

  componentDidUpdate(prevProps: ErrorBoundaryProps) {
    const { resetOnPropsChange = true } = this.props;
    
    // Reset error boundary when props change (if enabled)
    if (resetOnPropsChange && this.state.hasError && prevProps.children !== this.props.children) {
      this.resetError();
    }
  }

  componentWillUnmount() {
    if (this.retryTimeoutId) {
      clearTimeout(this.retryTimeoutId);
    }
  }

  private reportError = (error: Error, errorInfo: ErrorInfo, errorId: string) => {
    // In a real application, you would send this to your error monitoring service
    // For now, we'll just log it
    try {
      const errorReport = {
        errorId,
        message: error.message,
        stack: error.stack,
        componentStack: errorInfo.componentStack,
        timestamp: new Date().toISOString(),
        url: window.location.href,
        userAgent: navigator.userAgent,
      };

      // Example: Send to monitoring service
      // fetch('/api/errors', {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify(errorReport),
      // });

      console.log('Error report prepared:', errorReport);
    } catch (reportError) {
      console.error('Failed to report error:', reportError);
    }
  };

  private retry = () => {
    const { maxRetries = 3 } = this.props;
    const { retryCount } = this.state;

    if (retryCount < maxRetries) {
      this.setState(prevState => ({
        retryCount: prevState.retryCount + 1,
      }));

      // Retry after a short delay
      this.retryTimeoutId = setTimeout(() => {
        this.resetError();
      }, 1000);
    }
  };

  private resetError = () => {
    this.setState({
      hasError: false,
      error: null,
      errorInfo: null,
      errorId: '',
      retryCount: 0,
    });
  };

  private copyErrorDetails = () => {
    const { error, errorInfo, errorId } = this.state;
    const errorDetails = {
      errorId,
      message: error?.message,
      stack: error?.stack,
      componentStack: errorInfo?.componentStack,
      timestamp: new Date().toISOString(),
    };

    navigator.clipboard?.writeText(JSON.stringify(errorDetails, null, 2))
      .then(() => {
        alert('Error details copied to clipboard');
      })
      .catch(() => {
        console.log('Error details:', errorDetails);
        alert('Error details logged to console');
      });
  };

  render() {
    const { hasError, error, errorInfo, retryCount } = this.state;
    const { children, fallback, maxRetries = 3 } = this.props;

    if (hasError) {
      // Use custom fallback if provided
      if (fallback) {
        return fallback;
      }

      // Default error UI
      return (
        <div className="min-h-screen bg-gray-900 text-white flex items-center justify-center p-4">
          <div className="max-w-2xl w-full bg-gray-800 rounded-lg shadow-lg p-6">
            <div className="flex items-center mb-4">
              <div className="flex-shrink-0">
                <div className="w-8 h-8 bg-red-600 rounded-full flex items-center justify-center">
                  <svg className="w-5 h-5 text-white" fill="currentColor" viewBox="0 0 20 20">
                    <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                  </svg>
                </div>
              </div>
              <div className="ml-3">
                <h1 className="text-lg font-semibold text-red-400">Something went wrong</h1>
                <p className="text-sm text-gray-300">An unexpected error occurred in the application</p>
              </div>
            </div>

            <div className="mb-4">
              <div className="bg-gray-700 rounded p-3 mb-3">
                <p className="text-sm text-gray-300 mb-1">Error Message:</p>
                <p className="text-red-300 font-mono text-sm">{error?.message || 'Unknown error'}</p>
              </div>

              {process.env.NODE_ENV === 'development' && errorInfo && (
                <details className="mb-3">
                  <summary className="text-sm text-gray-400 cursor-pointer hover:text-gray-300">
                    Technical Details (Development)
                  </summary>
                  <div className="mt-2 bg-gray-700 rounded p-3">
                    <pre className="text-xs text-gray-300 whitespace-pre-wrap overflow-auto max-h-40">
                      {error?.stack}
                    </pre>
                    <div className="mt-2">
                      <p className="text-xs text-gray-400 mb-1">Component Stack:</p>
                      <pre className="text-xs text-gray-300 whitespace-pre-wrap overflow-auto max-h-20">
                        {errorInfo.componentStack}
                      </pre>
                    </div>
                  </div>
                </details>
              )}
            </div>

            <div className="flex flex-wrap gap-2">
              <button
                onClick={this.retry}
                disabled={retryCount >= maxRetries}
                className={`px-4 py-2 rounded text-sm font-medium transition-colors ${
                  retryCount >= maxRetries
                    ? 'bg-gray-600 text-gray-400 cursor-not-allowed'
                    : 'bg-blue-600 text-white hover:bg-blue-700'
                }`}
              >
                {retryCount >= maxRetries ? 'Max Retries Reached' : `Retry (${retryCount}/${maxRetries})`}
              </button>

              <button
                onClick={this.resetError}
                className="px-4 py-2 bg-green-600 text-white rounded text-sm font-medium hover:bg-green-700 transition-colors"
              >
                Reset
              </button>

              <button
                onClick={this.copyErrorDetails}
                className="px-4 py-2 bg-gray-600 text-white rounded text-sm font-medium hover:bg-gray-700 transition-colors"
              >
                Copy Error Details
              </button>

              <button
                onClick={() => window.location.reload()}
                className="px-4 py-2 bg-purple-600 text-white rounded text-sm font-medium hover:bg-purple-700 transition-colors"
              >
                Reload Page
              </button>
            </div>

            <div className="mt-4 text-xs text-gray-400">
              <p>If this error persists, please contact support with the error details.</p>
              <p>Error ID: {this.state.errorId}</p>
            </div>
          </div>
        </div>
      );
    }

    return children;
  }
}

export default ErrorBoundary;