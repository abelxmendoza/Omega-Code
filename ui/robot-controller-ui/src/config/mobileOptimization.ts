/**
 * Mobile Network Optimization Configuration
 * 
 * This file contains settings optimized for mobile hotspot usage,
 * including connection timeouts, retry strategies, and bandwidth management.
 */

export interface MobileOptimizationConfig {
  // Connection settings
  connectionTimeout: number;
  maxRetries: number;
  retryDelay: number;
  maxRetryDelay: number;
  
  // WebSocket settings
  pingInterval: number;
  pongTimeout: number;
  heartbeatInterval: number;
  
  // Bandwidth optimization
  enableCompression: boolean;
  maxMessageSize: number;
  batchMessages: boolean;
  batchDelay: number;
  
  // Video streaming
  videoQuality: 'low' | 'medium' | 'high' | 'auto';
  videoBitrate: number;
  videoFramerate: number;
  
  // Update frequencies
  performanceUpdateInterval: number;
  sensorUpdateInterval: number;
  statusUpdateInterval: number;
  
  // Mobile-specific
  enableOfflineMode: boolean;
  cacheExpiration: number;
  preloadResources: boolean;
}

// Default configuration for mobile networks
export const MOBILE_CONFIG: MobileOptimizationConfig = {
  // Connection settings (more aggressive for mobile)
  connectionTimeout: 10000, // 10 seconds
  maxRetries: 5,
  retryDelay: 2000, // 2 seconds base delay
  maxRetryDelay: 30000, // 30 seconds max delay
  
  // WebSocket settings
  pingInterval: 30000, // 30 seconds
  pongTimeout: 10000, // 10 seconds
  heartbeatInterval: 15000, // 15 seconds
  
  // Bandwidth optimization
  enableCompression: true,
  maxMessageSize: 1024 * 1024, // 1MB
  batchMessages: true,
  batchDelay: 100, // 100ms
  
  // Video streaming (conservative for mobile)
  videoQuality: 'auto',
  videoBitrate: 500000, // 500kbps
  videoFramerate: 15, // 15 FPS
  
  // Update frequencies (reduced for mobile)
  performanceUpdateInterval: 2000, // 2 seconds
  sensorUpdateInterval: 1000, // 1 second
  statusUpdateInterval: 500, // 500ms
  
  // Mobile-specific
  enableOfflineMode: true,
  cacheExpiration: 300000, // 5 minutes
  preloadResources: true,
};

// WiFi configuration (less aggressive)
export const WIFI_CONFIG: MobileOptimizationConfig = {
  // Connection settings
  connectionTimeout: 5000, // 5 seconds
  maxRetries: 3,
  retryDelay: 1000, // 1 second base delay
  maxRetryDelay: 10000, // 10 seconds max delay
  
  // WebSocket settings
  pingInterval: 30000, // 30 seconds
  pongTimeout: 5000, // 5 seconds
  heartbeatInterval: 10000, // 10 seconds
  
  // Bandwidth optimization
  enableCompression: false,
  maxMessageSize: 2 * 1024 * 1024, // 2MB
  batchMessages: false,
  batchDelay: 0,
  
  // Video streaming (higher quality for WiFi)
  videoQuality: 'high',
  videoBitrate: 2000000, // 2Mbps
  videoFramerate: 30, // 30 FPS
  
  // Update frequencies (faster for WiFi)
  performanceUpdateInterval: 1000, // 1 second
  sensorUpdateInterval: 500, // 500ms
  statusUpdateInterval: 250, // 250ms
  
  // Mobile-specific
  enableOfflineMode: false,
  cacheExpiration: 60000, // 1 minute
  preloadResources: false,
};

// Connection quality-based configurations
export const CONNECTION_CONFIGS = {
  excellent: WIFI_CONFIG,
  good: {
    ...MOBILE_CONFIG,
    videoBitrate: 1000000, // 1Mbps
    videoFramerate: 20,
    performanceUpdateInterval: 1500,
  },
  fair: {
    ...MOBILE_CONFIG,
    videoBitrate: 500000, // 500kbps
    videoFramerate: 15,
    performanceUpdateInterval: 2000,
  },
  poor: {
    ...MOBILE_CONFIG,
    videoBitrate: 250000, // 250kbps
    videoFramerate: 10,
    performanceUpdateInterval: 3000,
    enableCompression: true,
    batchMessages: true,
  },
} as const;

// Utility functions
export function getOptimizedConfig(
  connectionType: 'wifi' | 'cellular' | 'unknown',
  connectionQuality: 'excellent' | 'good' | 'fair' | 'poor' | 'unknown'
): MobileOptimizationConfig {
  // Use connection quality if available
  if (connectionQuality !== 'unknown' && connectionQuality in CONNECTION_CONFIGS) {
    return CONNECTION_CONFIGS[connectionQuality];
  }
  
  // Fall back to connection type
  return connectionType === 'wifi' ? WIFI_CONFIG : MOBILE_CONFIG;
}

export function shouldEnableVideoStream(config: MobileOptimizationConfig): boolean {
  return config.videoBitrate > 0 && config.videoFramerate > 0;
}

export function getRecommendedSettings(config: MobileOptimizationConfig): string[] {
  const recommendations: string[] = [];
  
  if (config.enableCompression) {
    recommendations.push('Enable data compression for better performance');
  }
  
  if (config.videoBitrate < 1000000) {
    recommendations.push('Consider reducing video quality for better stability');
  }
  
  if (config.performanceUpdateInterval > 1500) {
    recommendations.push('Performance updates are slowed for better connection stability');
  }
  
  if (config.maxRetries > 3) {
    recommendations.push('Increased retry attempts for unreliable connections');
  }
  
  return recommendations;
}

// Environment-based configuration
export function getEnvironmentConfig(): MobileOptimizationConfig {
  const isMobile = /Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
  const hasSlowConnection = 'connection' in navigator && 
    (navigator as any).connection?.effectiveType === 'slow-2g';
  
  if (hasSlowConnection) {
    return CONNECTION_CONFIGS.poor;
  }
  
  if (isMobile) {
    return MOBILE_CONFIG;
  }
  
  return WIFI_CONFIG;
}
