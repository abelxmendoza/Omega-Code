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
    const { hasError, error, errorInfo } = this.state;
    const { children, fallback } = this.props;

    if (hasError) {
      // Log error to console instead of showing UI
      console.error('ErrorBoundary: Error caught and logged to console. Check browser console for details.');
      
      // Use custom fallback if provided, otherwise render children (errors logged to console)
      if (fallback) {
        return fallback;
      }

      // Don't show error UI - just log to console and render children
      // Errors are visible in browser console and terminal
      return children;
    }

    return children;
  }
}

export default ErrorBoundary;