/**
 * Connection Health Monitor for Mobile/Hotspot Usage
 * 
 * Features:
 * - Connection quality assessment
 * - Automatic retry with exponential backoff
 * - Mobile network optimization
 * - Connection stability tracking
 * - Bandwidth-aware fallbacks
 */

export interface ConnectionMetrics {
  latency: number;
  bandwidth: number;
  stability: number;
  packetLoss: number;
  connectionType: 'wifi' | 'cellular' | 'unknown';
  signalStrength?: number;
}

export interface ConnectionHealth {
  status: 'excellent' | 'good' | 'fair' | 'poor' | 'disconnected';
  metrics: ConnectionMetrics;
  recommendations: string[];
  lastUpdated: number;
}

class ConnectionHealthMonitor {
  private metrics: ConnectionMetrics = {
    latency: 0,
    bandwidth: 0,
    stability: 0,
    packetLoss: 0,
    connectionType: 'unknown'
  };
  
  private healthHistory: ConnectionHealth[] = [];
  private isMonitoring = false;
  private pingInterval: NodeJS.Timeout | null = null;
  
  // Mobile network optimization settings
  private readonly MOBILE_SETTINGS = {
    maxRetries: 5,
    baseDelay: 1000,
    maxDelay: 30000,
    timeoutMs: 10000,
    pingInterval: 5000,
    bandwidthThreshold: 1000000, // 1MB/s
    latencyThreshold: 200, // 200ms
  };

  async startMonitoring(): Promise<void> {
    if (this.isMonitoring) return;
    
    this.isMonitoring = true;
    console.log('[ConnectionHealth] Starting mobile network monitoring');
    
    // Detect connection type
    await this.detectConnectionType();
    
    // Start periodic health checks
    this.pingInterval = setInterval(() => {
      this.performHealthCheck();
    }, this.MOBILE_SETTINGS.pingInterval);
    
    // Initial health check
    await this.performHealthCheck();
  }

  stopMonitoring(): void {
    if (!this.isMonitoring) return;
    
    this.isMonitoring = false;
    if (this.pingInterval) {
      clearInterval(this.pingInterval);
      this.pingInterval = null;
    }
    
    console.log('[ConnectionHealth] Stopped monitoring');
  }

  private async detectConnectionType(): Promise<void> {
    try {
      // Use Network Information API if available
      if ('connection' in navigator) {
        const connection = (navigator as any).connection;
        if (connection) {
          this.metrics.connectionType = connection.effectiveType === '4g' || connection.effectiveType === '3g' 
            ? 'cellular' 
            : 'wifi';
          this.metrics.signalStrength = connection.rtt;
        }
      }
      
      // Fallback: detect based on user agent or network characteristics
      if (this.metrics.connectionType === 'unknown') {
        // Simple heuristic: mobile devices often use cellular
        const isMobile = /Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
        this.metrics.connectionType = isMobile ? 'cellular' : 'wifi';
      }
    } catch (error) {
      console.warn('[ConnectionHealth] Failed to detect connection type:', error);
    }
  }

  private async performHealthCheck(): Promise<void> {
    try {
      const startTime = performance.now();
      
      // Test latency with a simple ping to the Pi
      const latency = await this.measureLatency();
      const bandwidth = await this.measureBandwidth();
      const stability = this.calculateStability();
      
      this.metrics = {
        ...this.metrics,
        latency,
        bandwidth,
        stability,
        packetLoss: this.calculatePacketLoss()
      };
      
      const health = this.assessHealth();
      this.healthHistory.push(health);
      
      // Keep only last 10 health records
      if (this.healthHistory.length > 10) {
        this.healthHistory.shift();
      }
      
      console.log('[ConnectionHealth] Health check complete:', health);
      
    } catch (error) {
      console.error('[ConnectionHealth] Health check failed:', error);
    }
  }

  private async measureLatency(): Promise<number> {
    try {
      const startTime = performance.now();
      
      // Try to ping the Pi's gateway server
      const response = await fetch('/api/performance-proxy/metrics', {
        method: 'GET',
        signal: AbortSignal.timeout(this.MOBILE_SETTINGS.timeoutMs)
      });
      
      if (response.ok) {
        const endTime = performance.now();
        return endTime - startTime;
      }
      
      return this.MOBILE_SETTINGS.latencyThreshold * 2; // Penalty for failed requests
    } catch (error) {
      return this.MOBILE_SETTINGS.latencyThreshold * 3; // High penalty for errors
    }
  }

  private async measureBandwidth(): Promise<number> {
    try {
      const startTime = performance.now();
      
      // Download a small test payload to measure bandwidth
      const response = await fetch('/api/performance-proxy/metrics', {
        method: 'GET',
        signal: AbortSignal.timeout(this.MOBILE_SETTINGS.timeoutMs)
      });
      
      if (response.ok) {
        const data = await response.text();
        const endTime = performance.now();
        const durationMs = endTime - startTime;
        const bytesPerSecond = (data.length * 1000) / durationMs;
        return bytesPerSecond;
      }
      
      return 0;
    } catch (error) {
      return 0;
    }
  }

  private calculateStability(): number {
    if (this.healthHistory.length < 2) return 100;
    
    const recent = this.healthHistory.slice(-5);
    const latencies = recent.map(h => h.metrics.latency);
    const avgLatency = latencies.reduce((a, b) => a + b, 0) / latencies.length;
    const variance = latencies.reduce((acc, lat) => acc + Math.pow(lat - avgLatency, 2), 0) / latencies.length;
    const stability = Math.max(0, 100 - (variance / avgLatency) * 100);
    
    return Math.round(stability);
  }

  private calculatePacketLoss(): number {
    if (this.healthHistory.length < 3) return 0;
    
    const recent = this.healthHistory.slice(-10);
    const failedChecks = recent.filter(h => h.status === 'disconnected').length;
    return (failedChecks / recent.length) * 100;
  }

  private assessHealth(): ConnectionHealth {
    const { latency, bandwidth, stability, packetLoss, connectionType } = this.metrics;
    
    let status: ConnectionHealth['status'] = 'excellent';
    const recommendations: string[] = [];
    
    // Assess based on mobile network characteristics
    if (connectionType === 'cellular') {
      if (latency > 500 || bandwidth < 100000 || stability < 50) {
        status = 'poor';
        recommendations.push('Consider switching to WiFi for better performance');
      } else if (latency > 300 || bandwidth < 500000 || stability < 70) {
        status = 'fair';
        recommendations.push('Connection may be unstable, consider reducing video quality');
      } else if (latency > 200 || bandwidth < 1000000 || stability < 85) {
        status = 'good';
        recommendations.push('Good connection, minor optimizations recommended');
      }
    } else {
      // WiFi assessment
      if (latency > 200 || bandwidth < 1000000 || stability < 70) {
        status = 'poor';
        recommendations.push('Check WiFi signal strength and router performance');
      } else if (latency > 100 || bandwidth < 5000000 || stability < 85) {
        status = 'fair';
        recommendations.push('WiFi connection could be improved');
      } else if (latency > 50 || bandwidth < 10000000 || stability < 95) {
        status = 'good';
        recommendations.push('Good WiFi connection');
      }
    }
    
    // Additional recommendations based on metrics
    if (packetLoss > 5) {
      status = 'poor';
      recommendations.push('High packet loss detected, check network stability');
    }
    
    if (latency > this.MOBILE_SETTINGS.latencyThreshold) {
      recommendations.push('High latency detected, consider reducing update frequency');
    }
    
    if (bandwidth < this.MOBILE_SETTINGS.bandwidthThreshold) {
      recommendations.push('Low bandwidth detected, consider disabling video stream');
    }
    
    return {
      status,
      metrics: this.metrics,
      recommendations,
      lastUpdated: Date.now()
    };
  }

  getCurrentHealth(): ConnectionHealth | null {
    return this.healthHistory[this.healthHistory.length - 1] || null;
  }

  getHealthHistory(): ConnectionHealth[] {
    return [...this.healthHistory];
  }

  getOptimizationSettings(): Record<string, any> {
    const health = this.getCurrentHealth();
    if (!health) return {};
    
    const settings: Record<string, any> = {};
    
    // Adjust settings based on connection quality
    switch (health.status) {
      case 'poor':
        settings.videoQuality = 'low';
        settings.updateFrequency = 2000; // 2 seconds
        settings.enableCompression = true;
        settings.maxRetries = 3;
        break;
      case 'fair':
        settings.videoQuality = 'medium';
        settings.updateFrequency = 1000; // 1 second
        settings.enableCompression = true;
        settings.maxRetries = 5;
        break;
      case 'good':
        settings.videoQuality = 'high';
        settings.updateFrequency = 500; // 500ms
        settings.enableCompression = false;
        settings.maxRetries = 5;
        break;
      case 'excellent':
        settings.videoQuality = 'high';
        settings.updateFrequency = 250; // 250ms
        settings.enableCompression = false;
        settings.maxRetries = 3;
        break;
    }
    
    return settings;
  }

  // Retry mechanism with exponential backoff
  async withRetry<T>(
    operation: () => Promise<T>,
    context: string = 'operation'
  ): Promise<T> {
    let lastError: Error;
    
    for (let attempt = 1; attempt <= this.MOBILE_SETTINGS.maxRetries; attempt++) {
      try {
        return await operation();
      } catch (error) {
        lastError = error as Error;
        console.warn(`[ConnectionHealth] ${context} failed (attempt ${attempt}/${this.MOBILE_SETTINGS.maxRetries}):`, error);
        
        if (attempt === this.MOBILE_SETTINGS.maxRetries) {
          break;
        }
        
        // Exponential backoff with jitter
        const delay = Math.min(
          this.MOBILE_SETTINGS.baseDelay * Math.pow(2, attempt - 1),
          this.MOBILE_SETTINGS.maxDelay
        );
        const jitter = Math.random() * 1000; // Add up to 1 second of jitter
        
        await new Promise(resolve => setTimeout(resolve, delay + jitter));
      }
    }
    
    throw lastError!;
  }
}

// Singleton instance
export const connectionHealth = new ConnectionHealthMonitor();

// Utility functions
export function getConnectionOptimizations(): Record<string, any> {
  return connectionHealth.getOptimizationSettings();
}

export function isMobileConnection(): boolean {
  const health = connectionHealth.getCurrentHealth();
  return health?.metrics.connectionType === 'cellular';
}

export function shouldReduceVideoQuality(): boolean {
  const health = connectionHealth.getCurrentHealth();
  return health?.status === 'poor' || health?.status === 'fair';
}

export function getRecommendedUpdateFrequency(): number {
  const optimizations = getConnectionOptimizations();
  return optimizations.updateFrequency || 1000;
}
