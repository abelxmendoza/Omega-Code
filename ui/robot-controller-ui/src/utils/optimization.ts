/*
# File: /src/utils/optimization.ts
# Summary:
# React optimization utilities for improved performance
# - Component memoization
# - Lazy loading helpers
# - Performance monitoring
# - Memory optimization
*/

import React, { memo, useMemo, useCallback, lazy, Suspense } from 'react';

// Simple debounce implementation
const debounce = <T extends (...args: any[]) => any>(
  func: T,
  wait: number
): ((...args: Parameters<T>) => void) => {
  let timeout: NodeJS.Timeout;
  return (...args: Parameters<T>) => {
    clearTimeout(timeout);
    timeout = setTimeout(() => func(...args), wait);
  };
};

// Simple throttle implementation
const throttle = <T extends (...args: any[]) => any>(
  func: T,
  limit: number
): ((...args: Parameters<T>) => void) => {
  let inThrottle: boolean;
  return (...args: Parameters<T>) => {
    if (!inThrottle) {
      func(...args);
      inThrottle = true;
      setTimeout(() => (inThrottle = false), limit);
    }
  };
};

// Performance monitoring
export const performanceMonitor = {
  measureRender: (componentName: string, renderFn: () => React.ReactNode) => {
    const start = performance.now();
    const result = renderFn();
    const end = performance.now();
    console.log(`${componentName} render time: ${end - start}ms`);
    return result;
  },

  measureAsync: async (operationName: string, asyncFn: () => Promise<any>) => {
    const start = performance.now();
    const result = await asyncFn();
    const end = performance.now();
    console.log(`${operationName} execution time: ${end - start}ms`);
    return result;
  }
};

// Optimized component wrapper
export const withOptimization = <P extends object>(
  Component: React.ComponentType<P>,
  options: {
    memoize?: boolean;
  } = {}
): React.ComponentType<P> => {
  if (options.memoize) {
    return memo(Component) as unknown as React.ComponentType<P>;
  }
  
  return Component;
};

// Lazy loading helper
export const createLazyComponent = <P extends object>(
  importFn: () => Promise<{ default: React.ComponentType<P> }>,
  fallback?: React.ReactNode
) => {
  const LazyComponent = lazy(importFn);
  
  return (props: P) => 
    React.createElement(
      Suspense,
      { fallback: fallback || React.createElement('div', null, 'Loading...') },
      React.createElement(LazyComponent, props as any)
    );
};

// Memory optimization hooks
export const useOptimizedCallback = <T extends (...args: any[]) => any>(
  callback: T,
  deps: React.DependencyList
): T => {
  return useCallback(callback, deps);
};

export const useOptimizedMemo = <T>(
  factory: () => T,
  deps: React.DependencyList
): T => {
  return useMemo(factory, deps);
};

// Debounced hook
export const useDebouncedCallback = <T extends (...args: any[]) => any>(
  callback: T,
  delay: number,
  deps: React.DependencyList
): T => {
  const debouncedCallback = useMemo(
    () => debounce(callback, delay),
    [delay, ...deps]
  );

  React.useEffect(() => {
    return () => {
      // Cleanup handled by debounce implementation
    };
  }, [debouncedCallback]);

  return debouncedCallback as T;
};

// Throttled hook
export const useThrottledCallback = <T extends (...args: any[]) => any>(
  callback: T,
  delay: number,
  deps: React.DependencyList
): T => {
  const throttledCallback = useMemo(
    () => throttle(callback, delay),
    [delay, ...deps]
  );

  React.useEffect(() => {
    return () => {
      // Cleanup handled by throttle implementation
    };
  }, [throttledCallback]);

  return throttledCallback as T;
};

// Virtual scrolling helper
export const useVirtualScrolling = <T>(
  items: T[],
  itemHeight: number,
  containerHeight: number
) => {
  const [scrollTop, setScrollTop] = React.useState(0);
  
  const visibleItems = useMemo(() => {
    const startIndex = Math.floor(scrollTop / itemHeight);
    const endIndex = Math.min(
      startIndex + Math.ceil(containerHeight / itemHeight) + 1,
      items.length
    );
    
    return items.slice(startIndex, endIndex).map((item, index) => ({
      item,
      index: startIndex + index,
      top: (startIndex + index) * itemHeight
    }));
  }, [items, itemHeight, containerHeight, scrollTop]);

  const totalHeight = items.length * itemHeight;

  return {
    visibleItems,
    totalHeight,
    setScrollTop
  };
};

// Image optimization
export const OptimizedImage: React.FC<{
  src: string;
  alt: string;
  width?: number;
  height?: number;
  className?: string;
  loading?: 'lazy' | 'eager';
}> = memo(({ src, alt, width, height, className, loading = 'lazy' }) => {
  const [isLoaded, setIsLoaded] = React.useState(false);
  const [hasError, setHasError] = React.useState(false);

  const handleLoad = useCallback(() => {
    setIsLoaded(true);
  }, []);

  const handleError = useCallback(() => {
    setHasError(true);
  }, []);

  return React.createElement(
    'div',
    { className: `relative ${className}` },
    !isLoaded && !hasError && React.createElement(
      'div',
      { className: 'absolute inset-0 bg-gray-200 animate-pulse' }
    ),
    hasError ? React.createElement(
      'div',
      { className: 'absolute inset-0 bg-gray-300 flex items-center justify-center' },
      React.createElement('span', { className: 'text-gray-500' }, 'Failed to load')
    ) : React.createElement('img', {
      src: src,
      alt: alt,
      width: width,
      height: height,
      loading: loading,
      onLoad: handleLoad,
      onError: handleError,
      className: `transition-opacity duration-300 ${
        isLoaded ? 'opacity-100' : 'opacity-0'
      }`
    })
  );
});

// Bundle size optimization
export const bundleOptimizer = {
  // Dynamic imports for code splitting
  loadComponent: async (componentPath: string) => {
    try {
      const module = await import(componentPath);
      return module.default;
    } catch (error) {
      console.error(`Failed to load component: ${componentPath}`, error);
      return null;
    }
  },

  // Preload critical components
  preloadComponent: (componentPath: string) => {
    const link = document.createElement('link');
    link.rel = 'modulepreload';
    link.href = componentPath;
    document.head.appendChild(link);
  }
};

// Performance metrics
export const performanceMetrics = {
  getMemoryUsage: () => {
    if ('memory' in performance) {
      return (performance as any).memory;
    }
    return null;
  },

  getNavigationTiming: () => {
    return performance.getEntriesByType('navigation')[0] as PerformanceNavigationTiming;
  },

  getResourceTiming: () => {
    return performance.getEntriesByType('resource');
  },

  measureCustomMetric: (name: string, startMark: string, endMark: string) => {
    try {
      performance.measure(name, startMark, endMark);
      const measure = performance.getEntriesByName(name)[0];
      return measure.duration;
    } catch (error) {
      console.error('Failed to measure custom metric:', error);
      return 0;
    }
  }
};
