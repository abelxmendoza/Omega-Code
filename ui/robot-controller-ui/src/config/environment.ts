/**
 * Environment Configuration
 * Centralized configuration with fallbacks to prevent breaking changes
 */

export interface EnvironmentConfig {
  // Network profiles
  networkProfile: 'tailscale' | 'lan' | 'local';
  
  // WebSocket URLs with fallbacks
  wsUrls: {
    movement: string[];
    ultrasonic: string[];
    lineTracker: string[];
    lighting: string[];
  };
  
  // Video stream URLs
  videoUrls: string[];
  
  // Debug settings
  debug: {
    wsDebug: boolean;
    mockWs: boolean;
    verboseLogging: boolean;
  };
  
  // Connection settings
  connection: {
    reconnectAttempts: number;
    reconnectDelay: number;
    heartbeatInterval: number;
    connectionTimeout: number;
  };
}

/**
 * Get environment variable with fallback
 */
function getEnvVar(key: string, fallback: string = ''): string {
  if (typeof window !== 'undefined') {
    return (window as any).__ENV__?.[key] || process.env[key] || fallback;
  }
  return process.env[key] || fallback;
}

/**
 * Get boolean environment variable
 */
function getBooleanEnvVar(key: string, fallback: boolean = false): boolean {
  const value = getEnvVar(key);
  if (value === '') return fallback;
  return value.toLowerCase() === 'true' || value === '1';
}

/**
 * Build WebSocket URL candidates with fallbacks
 */
function buildWsCandidates(baseKey: string): string[] {
  const candidates: string[] = [];
  
  // Try Tailscale first
  const tailscale = getEnvVar(`${baseKey}_TAILSCALE`);
  if (tailscale) candidates.push(tailscale);
  
  // Then LAN
  const lan = getEnvVar(`${baseKey}_LAN`);
  if (lan) candidates.push(lan);
  
  // Then local fallbacks
  const local = getEnvVar(`${baseKey}_LOCAL`);
  if (local) candidates.push(local);
  
  // Add default localhost fallbacks
  if (baseKey.includes('MOVEMENT')) {
    candidates.push('ws://localhost:8081/', 'ws://localhost:3001/ws/movement');
  } else if (baseKey.includes('ULTRASONIC')) {
    candidates.push('ws://localhost:8080/ultrasonic', 'ws://localhost:3001/ws/ultrasonic');
  } else if (baseKey.includes('LINE_TRACKER')) {
    candidates.push('ws://localhost:8090/line-tracker', 'ws://localhost:3001/ws/line');
  } else if (baseKey.includes('LIGHTING')) {
    candidates.push('ws://localhost:8082/lighting', 'ws://localhost:3001/ws/lighting');
  }
  
  return candidates.filter((url, index, arr) => arr.indexOf(url) === index); // Remove duplicates
}

/**
 * Build video URL candidates
 */
function buildVideoCandidates(): string[] {
  const candidates: string[] = [];
  
  // Try Tailscale first
  const tailscale = getEnvVar('NEXT_PUBLIC_VIDEO_STREAM_URL_TAILSCALE');
  if (tailscale) candidates.push(tailscale);
  
  // Then LAN
  const lan = getEnvVar('NEXT_PUBLIC_VIDEO_STREAM_URL_LAN');
  if (lan) candidates.push(lan);
  
  // Then local
  const local = getEnvVar('NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL');
  if (local) candidates.push(local);
  
  // Add default fallbacks
  candidates.push(
    'http://localhost:5000/video_feed',
    'http://localhost:3001/api/video-proxy?profile=local'
  );
  
  return candidates.filter((url, index, arr) => arr.indexOf(url) === index);
}

/**
 * Get the current environment configuration
 */
export function getEnvironmentConfig(): EnvironmentConfig {
  const networkProfile = (getEnvVar('NEXT_PUBLIC_NETWORK_PROFILE', 'local') as 'tailscale' | 'lan' | 'local');
  
  return {
    networkProfile,
    
    wsUrls: {
      movement: buildWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT'),
      ultrasonic: buildWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC'),
      lineTracker: buildWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER'),
      lighting: buildWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING'),
    },
    
    videoUrls: buildVideoCandidates(),
    
    debug: {
      wsDebug: getBooleanEnvVar('NEXT_PUBLIC_WS_DEBUG'),
      mockWs: getBooleanEnvVar('NEXT_PUBLIC_MOCK_WS'),
      verboseLogging: getBooleanEnvVar('NEXT_PUBLIC_VERBOSE_LOGGING'),
    },
    
    connection: {
      reconnectAttempts: parseInt(getEnvVar('NEXT_PUBLIC_RECONNECT_ATTEMPTS', '5')),
      reconnectDelay: parseInt(getEnvVar('NEXT_PUBLIC_RECONNECT_DELAY', '3000')),
      heartbeatInterval: parseInt(getEnvVar('NEXT_PUBLIC_HEARTBEAT_INTERVAL', '10000')),
      connectionTimeout: parseInt(getEnvVar('NEXT_PUBLIC_CONNECTION_TIMEOUT', '5000')),
    },
  };
}

/**
 * Log configuration for debugging
 */
export function logEnvironmentConfig(config: EnvironmentConfig): void {
  if (!config.debug.verboseLogging) return;
  
  console.group('🔧 Environment Configuration');
  console.log('Network Profile:', config.networkProfile);
  console.log('WebSocket URLs:', config.wsUrls);
  console.log('Video URLs:', config.videoUrls);
  console.log('Debug Settings:', config.debug);
  console.log('Connection Settings:', config.connection);
  console.groupEnd();
}

// Export the configuration instance
export const envConfig = getEnvironmentConfig();

// Log configuration in development
if (envConfig.debug.verboseLogging) {
  logEnvironmentConfig(envConfig);
}

