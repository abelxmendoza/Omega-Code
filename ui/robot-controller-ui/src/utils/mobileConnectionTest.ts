/**
 * Mobile Connection Test Utility
 * 
 * This utility helps test and validate mobile hotspot connections
 * by performing various network tests and providing recommendations.
 */

export interface ConnectionTestResult {
  success: boolean;
  latency: number;
  bandwidth: number;
  stability: number;
  errors: string[];
  recommendations: string[];
  testDuration: number;
}

export interface NetworkTestOptions {
  timeout?: number;
  testDuration?: number;
  testBandwidth?: boolean;
  testStability?: boolean;
  verbose?: boolean;
}

class MobileConnectionTester {
  private readonly DEFAULT_TIMEOUT = 10000;
  private readonly DEFAULT_TEST_DURATION = 30000; // 30 seconds

  async testConnection(options: NetworkTestOptions = {}): Promise<ConnectionTestResult> {
    const {
      timeout = this.DEFAULT_TIMEOUT,
      testDuration = this.DEFAULT_TEST_DURATION,
      testBandwidth = true,
      testStability = true,
      verbose = false
    } = options;

    const startTime = Date.now();
    const errors: string[] = [];
    const recommendations: string[] = [];
    
    let latency = 0;
    let bandwidth = 0;
    let stability = 0;

    if (verbose) {
      console.log('[MobileConnectionTest] Starting connection test...');
    }

    try {
      // Test 1: Basic connectivity and latency
      latency = await this.testLatency(timeout);
      if (verbose) {
        console.log(`[MobileConnectionTest] Latency: ${latency}ms`);
      }

      // Test 2: Bandwidth (if enabled)
      if (testBandwidth) {
        bandwidth = await this.testBandwidth(timeout);
        if (verbose) {
          console.log(`[MobileConnectionTest] Bandwidth: ${(bandwidth / 1000).toFixed(0)}KB/s`);
        }
      }

      // Test 3: Stability (if enabled)
      if (testStability) {
        stability = await this.testStability(testDuration, timeout);
        if (verbose) {
          console.log(`[MobileConnectionTest] Stability: ${stability}%`);
        }
      }

      // Generate recommendations
      recommendations.push(...this.generateRecommendations(latency, bandwidth, stability));

    } catch (error) {
      errors.push(`Connection test failed: ${error}`);
      if (verbose) {
        console.error('[MobileConnectionTest] Test failed:', error);
      }
    }

    const testDuration_ms = Date.now() - startTime;
    const success = errors.length === 0 && latency > 0;

    if (verbose) {
      console.log(`[MobileConnectionTest] Test completed in ${testDuration_ms}ms`);
    }

    return {
      success,
      latency,
      bandwidth,
      stability,
      errors,
      recommendations,
      testDuration: testDuration_ms
    };
  }

  private async testLatency(timeout: number): Promise<number> {
    const startTime = performance.now();
    
    try {
      const response = await fetch('/api/performance-proxy/metrics', {
        method: 'GET',
        signal: AbortSignal.timeout(timeout)
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const endTime = performance.now();
      return endTime - startTime;
    } catch (error) {
      throw new Error(`Latency test failed: ${error}`);
    }
  }

  private async testBandwidth(timeout: number): Promise<number> {
    const startTime = performance.now();
    
    try {
      const response = await fetch('/api/performance-proxy/metrics', {
        method: 'GET',
        signal: AbortSignal.timeout(timeout)
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.text();
      const endTime = performance.now();
      const durationMs = endTime - startTime;
      
      // Calculate bytes per second
      return (data.length * 1000) / durationMs;
    } catch (error) {
      throw new Error(`Bandwidth test failed: ${error}`);
    }
  }

  private async testStability(testDuration: number, timeout: number): Promise<number> {
    const testInterval = 2000; // Test every 2 seconds
    const maxTests = Math.floor(testDuration / testInterval);
    const results: number[] = [];
    
    for (let i = 0; i < maxTests; i++) {
      try {
        const latency = await this.testLatency(timeout);
        results.push(latency);
        
        // Wait for next test
        if (i < maxTests - 1) {
          await new Promise(resolve => setTimeout(resolve, testInterval));
        }
      } catch (error) {
        // Failed test counts as high latency
        results.push(timeout);
      }
    }

    if (results.length === 0) {
      return 0;
    }

    // Calculate stability based on latency variance
    const avgLatency = results.reduce((a, b) => a + b, 0) / results.length;
    const variance = results.reduce((acc, lat) => acc + Math.pow(lat - avgLatency, 2), 0) / results.length;
    const stability = Math.max(0, 100 - (variance / avgLatency) * 100);
    
    return Math.round(stability);
  }

  private generateRecommendations(latency: number, bandwidth: number, stability: number): string[] {
    const recommendations: string[] = [];

    // Latency recommendations
    if (latency > 500) {
      recommendations.push('High latency detected - consider switching to WiFi or moving closer to router');
    } else if (latency > 200) {
      recommendations.push('Moderate latency - video streaming may be choppy');
    } else if (latency < 50) {
      recommendations.push('Excellent latency - optimal for real-time control');
    }

    // Bandwidth recommendations
    if (bandwidth > 0) {
      if (bandwidth < 100000) { // < 100KB/s
        recommendations.push('Very low bandwidth - disable video streaming and reduce update frequency');
      } else if (bandwidth < 500000) { // < 500KB/s
        recommendations.push('Low bandwidth - consider reducing video quality');
      } else if (bandwidth > 2000000) { // > 2MB/s
        recommendations.push('High bandwidth - can enable high-quality video streaming');
      }
    }

    // Stability recommendations
    if (stability < 50) {
      recommendations.push('Poor connection stability - enable automatic reconnection and reduce update frequency');
    } else if (stability < 80) {
      recommendations.push('Moderate stability - consider enabling compression and batching');
    } else if (stability > 95) {
      recommendations.push('Excellent stability - optimal for real-time robot control');
    }

    // General mobile recommendations
    if (this.isMobileDevice()) {
      recommendations.push('Mobile device detected - using optimized settings for cellular networks');
    }

    return recommendations;
  }

  private isMobileDevice(): boolean {
    return /Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
  }

  // Quick connection test for UI status
  async quickTest(): Promise<{ success: boolean; latency: number; error?: string }> {
    try {
      const latency = await this.testLatency(5000); // 5 second timeout
      return { success: true, latency };
    } catch (error) {
      return { success: false, latency: 0, error: String(error) };
    }
  }

  // Test WebSocket connection specifically
  async testWebSocketConnection(url: string, timeout: number = 10000): Promise<boolean> {
    return new Promise((resolve) => {
      const ws = new WebSocket(url);
      let resolved = false;

      const cleanup = () => {
        if (!resolved) {
          resolved = true;
          ws.close();
        }
      };

      const timeoutId = setTimeout(() => {
        cleanup();
        resolve(false);
      }, timeout);

      ws.onopen = () => {
        cleanup();
        clearTimeout(timeoutId);
        resolve(true);
      };

      ws.onerror = () => {
        cleanup();
        clearTimeout(timeoutId);
        resolve(false);
      };

      ws.onclose = () => {
        if (!resolved) {
          cleanup();
          clearTimeout(timeoutId);
          resolve(false);
        }
      };
    });
  }

  // Test multiple WebSocket endpoints
  async testAllWebSocketConnections(urls: string[]): Promise<Record<string, boolean>> {
    const results: Record<string, boolean> = {};
    
    const promises = urls.map(async (url) => {
      const success = await this.testWebSocketConnection(url);
      results[url] = success;
      return { url, success };
    });

    await Promise.allSettled(promises);
    return results;
  }
}

// Singleton instance
export const mobileConnectionTester = new MobileConnectionTester();

// Utility functions
export async function testMobileConnection(options?: NetworkTestOptions): Promise<ConnectionTestResult> {
  return mobileConnectionTester.testConnection(options);
}

export async function quickConnectionTest(): Promise<{ success: boolean; latency: number; error?: string }> {
  return mobileConnectionTester.quickTest();
}

export async function testWebSocketEndpoints(urls: string[]): Promise<Record<string, boolean>> {
  return mobileConnectionTester.testAllWebSocketConnections(urls);
}
