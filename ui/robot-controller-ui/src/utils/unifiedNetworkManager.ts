/**
 * Unified Network Manager
 * 
 * Handles seamless switching between WiFi, Tailscale, and mobile hotspot
 * with automatic fallback and optimization for each network type.
 */

export interface NetworkProfile {
  id: string;
  name: string;
  type: 'wifi' | 'tailscale' | 'mobile' | 'direct';
  priority: number;
  gatewayHost: string;
  gatewayPort: string;
  wsEndpoints: {
    movement: string;
    ultrasonic: string;
    lineTracker: string;
    lighting: string;
    video: string;
  };
  httpEndpoints: {
    gateway: string;
    performance: string;
    video: string;
  };
  optimization: {
    timeout: number;
    retries: number;
    compression: boolean;
    videoQuality: 'low' | 'medium' | 'high';
    updateFrequency: number;
  };
  isAvailable: boolean;
  latency?: number;
  bandwidth?: number;
  stability?: number;
}

export interface NetworkTestResult {
  profile: NetworkProfile;
  success: boolean;
  latency: number;
  bandwidth: number;
  stability: number;
  error?: string;
  testDuration: number;
}

class UnifiedNetworkManager {
  private profiles: NetworkProfile[] = [];
  private currentProfile: NetworkProfile | null = null;
  private testResults: Map<string, NetworkTestResult> = new Map();
  private isTesting = false;
  private fallbackInProgress = false;
  private listeners: Set<(profile: NetworkProfile) => void> = new Set();

  constructor() {
    this.initializeProfiles();
    this.startPeriodicTesting();
  }

  private initializeProfiles(): void {
    // Tailscale Profile (Highest Priority)
    this.profiles.push({
      id: 'tailscale',
      name: 'Tailscale VPN',
      type: 'tailscale',
      priority: 1,
      gatewayHost: 'omega1-1.hartley-ghost.ts.net',
      gatewayPort: '7070',
      wsEndpoints: {
        movement: 'ws://omega1-1.hartley-ghost.ts.net:8081/',
        ultrasonic: 'ws://omega1-1.hartley-ghost.ts.net:8080/ultrasonic',
        lineTracker: 'ws://omega1-1.hartley-ghost.ts.net:7070/ws/line',
        lighting: 'ws://100.93.225.61:8082/lighting',
        video: 'ws://omega1-1.hartley-ghost.ts.net:7070/video_feed'
      },
      httpEndpoints: {
        gateway: 'http://omega1-1.hartley-ghost.ts.net:7070',
        performance: 'http://omega1-1.hartley-ghost.ts.net:7070/api/performance/metrics',
        video: 'http://omega1-1.hartley-ghost.ts.net:7070/video_feed'
      },
      optimization: {
        timeout: 10000, // Increased timeout
        retries: 3,
        compression: false,
        videoQuality: 'high',
        updateFrequency: 250
      },
      isAvailable: false
    });

    // WiFi Profile (High Priority)
    this.profiles.push({
      id: 'wifi',
      name: 'WiFi Network',
      type: 'wifi',
      priority: 2,
      gatewayHost: '192.168.6.164',
      gatewayPort: '7070',
      wsEndpoints: {
        movement: 'ws://192.168.1.107:8081/',
        ultrasonic: 'ws://192.168.1.107:8080/ultrasonic',
        lineTracker: 'ws://192.168.6.164:7070/ws/line',
        lighting: 'ws://192.168.1.107:8082/lighting',
        video: 'ws://192.168.6.164:7070/video_feed'
      },
      httpEndpoints: {
        gateway: 'http://192.168.6.164:7070',
        performance: 'http://192.168.6.164:7070/api/performance/metrics',
        video: 'http://192.168.6.164:7070/video_feed'
      },
      optimization: {
        timeout: 10000,
        retries: 3,
        compression: false,
        videoQuality: 'high',
        updateFrequency: 500
      },
      isAvailable: false
    });

    // Mobile Hotspot Profile (Medium Priority)
    this.profiles.push({
      id: 'mobile',
      name: 'Mobile Hotspot',
      type: 'mobile',
      priority: 3,
      gatewayHost: '192.168.6.164', // Same as WiFi but with mobile optimizations
      gatewayPort: '7070',
      wsEndpoints: {
        movement: 'ws://192.168.1.107:8081/',
        ultrasonic: 'ws://192.168.1.107:8080/ultrasonic',
        lineTracker: 'ws://192.168.6.164:7070/ws/line',
        lighting: 'ws://192.168.1.107:8082/lighting',
        video: 'ws://192.168.6.164:7070/video_feed'
      },
      httpEndpoints: {
        gateway: 'http://192.168.6.164:7070',
        performance: 'http://192.168.6.164:7070/api/performance/metrics',
        video: 'http://192.168.6.164:7070/video_feed'
      },
      optimization: {
        timeout: 10000,
        retries: 5,
        compression: true,
        videoQuality: 'medium',
        updateFrequency: 1000
      },
      isAvailable: false
    });

    // Direct Tailscale Profile (Fallback)
    this.profiles.push({
      id: 'tailscale-direct',
      name: 'Tailscale Direct',
      type: 'tailscale',
      priority: 4,
      gatewayHost: '100.93.225.61', // Direct Tailscale IP
      gatewayPort: '7070',
      wsEndpoints: {
        movement: 'ws://100.93.225.61:8081/',
        ultrasonic: 'ws://100.93.225.61:8080/ultrasonic',
        lineTracker: 'ws://100.93.225.61:7070/ws/line',
        lighting: 'ws://100.93.225.61:8082/lighting',
        video: 'ws://100.93.225.61:7070/video_feed'
      },
      httpEndpoints: {
        gateway: 'http://100.93.225.61:7070',
        performance: 'http://100.93.225.61:7070/api/performance/metrics',
        video: 'http://100.93.225.61:7070/video_feed'
      },
      optimization: {
        timeout: 10000,
        retries: 4,
        compression: true,
        videoQuality: 'medium',
        updateFrequency: 750
      },
      isAvailable: false
    });

    // Local Profile (Lowest Priority)
    this.profiles.push({
      id: 'local',
      name: 'Local Network',
      type: 'direct',
      priority: 5,
      gatewayHost: 'localhost',
      gatewayPort: '7070',
      wsEndpoints: {
        movement: 'ws://localhost:8081/',
        ultrasonic: 'ws://localhost:8080/ultrasonic',
        lineTracker: 'ws://localhost:7070/ws/line',
        lighting: 'ws://localhost:8082/lighting',
        video: 'ws://localhost:7070/video_feed'
      },
      httpEndpoints: {
        gateway: 'http://localhost:7070',
        performance: 'http://localhost:7070/api/performance/metrics',
        video: 'http://localhost:7070/video_feed'
      },
      optimization: {
        timeout: 10000,
        retries: 2,
        compression: false,
        videoQuality: 'high',
        updateFrequency: 250
      },
      isAvailable: false
    });

    // Sort by priority
    this.profiles.sort((a, b) => a.priority - b.priority);
  }

  async startPeriodicTesting(): Promise<void> {
    // Test all profiles every 30 seconds
    setInterval(async () => {
      if (!this.isTesting) {
        await this.testAllProfiles();
      }
    }, 30000);

    // Initial test
    await this.testAllProfiles();
  }

  async testAllProfiles(): Promise<void> {
    if (this.isTesting) return;
    
    this.isTesting = true;
    console.log('[UnifiedNetworkManager] Testing all network profiles...');

    const testPromises = this.profiles.map(profile => this.testProfile(profile));
    const results = await Promise.allSettled(testPromises);

    // Update availability based on test results
    results.forEach((result, index) => {
      const profile = this.profiles[index];
      if (result.status === 'fulfilled' && result.value.success) {
        profile.isAvailable = true;
        profile.latency = result.value.latency;
        profile.bandwidth = result.value.bandwidth;
        profile.stability = result.value.stability;
        this.testResults.set(profile.id, result.value);
      } else {
        profile.isAvailable = false;
        console.warn(`[UnifiedNetworkManager] Profile ${profile.name} is unavailable`);
      }
    });

    // Auto-select best available profile
    await this.selectBestProfile();
    
    this.isTesting = false;
  }

  private async testProfile(profile: NetworkProfile): Promise<NetworkTestResult> {
    const startTime = Date.now();
    
    try {
      console.log(`[UnifiedNetworkManager] Testing profile: ${profile.name} (${profile.httpEndpoints.performance})`);
      
      // Test HTTP connectivity first
      const response = await fetch(profile.httpEndpoints.performance, {
        method: 'GET',
        signal: AbortSignal.timeout(profile.optimization.timeout)
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const endTime = Date.now();
      const latency = endTime - startTime;

      // Test bandwidth
      const data = await response.text();
      const bandwidth = (data.length * 1000) / latency;

      console.log(`[UnifiedNetworkManager] Profile ${profile.name} test successful: ${latency}ms latency`);

      return {
        profile,
        success: true,
        latency,
        bandwidth,
        stability: 100, // Simplified for now
        testDuration: Date.now() - startTime
      };

    } catch (error) {
      console.warn(`[UnifiedNetworkManager] Profile ${profile.name} test failed:`, error);
      return {
        profile,
        success: false,
        latency: 0,
        bandwidth: 0,
        stability: 0,
        error: String(error),
        testDuration: Date.now() - startTime
      };
    }
  }

  private async pingProfile(profile: NetworkProfile): Promise<number> {
    const startTime = performance.now();
    
    const response = await fetch(profile.httpEndpoints.performance, {
      method: 'GET',
      signal: AbortSignal.timeout(profile.optimization.timeout)
    });

    if (!response.ok) {
      throw new Error(`Ping failed: ${response.status}`);
    }

    return performance.now() - startTime;
  }

  private async selectBestProfile(): Promise<void> {
    const availableProfiles = this.profiles.filter(p => p.isAvailable);
    
    if (availableProfiles.length === 0) {
      console.warn('[UnifiedNetworkManager] No available profiles found');
      return;
    }

    // Find best profile based on priority and performance
    let bestProfile = availableProfiles[0];
    
    for (const profile of availableProfiles) {
      const testResult = this.testResults.get(profile.id);
      
      if (testResult && testResult.success) {
        // Prefer profiles with better performance metrics
        if (profile.priority < bestProfile.priority || 
            (profile.priority === bestProfile.priority && 
             testResult.latency < (this.testResults.get(bestProfile.id)?.latency || Infinity))) {
          bestProfile = profile;
        }
      }
    }

    // Switch to best profile if different
    if (!this.currentProfile || this.currentProfile.id !== bestProfile.id) {
      console.log(`[UnifiedNetworkManager] Switching to ${bestProfile.name} (${bestProfile.type})`);
      this.currentProfile = bestProfile;
      this.notifyListeners(bestProfile);
    }
  }

  async forceFallback(): Promise<NetworkProfile | null> {
    if (this.fallbackInProgress) return this.currentProfile;
    
    this.fallbackInProgress = true;
    console.log('[UnifiedNetworkManager] Forcing fallback to next available profile...');

    try {
      // Test all profiles and find next best
      await this.testAllProfiles();
      
      const availableProfiles = this.profiles.filter(p => p.isAvailable);
      const currentIndex = availableProfiles.findIndex(p => p.id === this.currentProfile?.id);
      
      // Select next available profile
      const nextProfile = availableProfiles[currentIndex + 1] || availableProfiles[0];
      
      if (nextProfile && nextProfile.id !== this.currentProfile?.id) {
        console.log(`[UnifiedNetworkManager] Fallback to ${nextProfile.name}`);
        this.currentProfile = nextProfile;
        this.notifyListeners(nextProfile);
      }
      
      return this.currentProfile;
    } finally {
      this.fallbackInProgress = false;
    }
  }

  getCurrentProfile(): NetworkProfile | null {
    return this.currentProfile;
  }

  getAllProfiles(): NetworkProfile[] {
    return [...this.profiles];
  }

  getAvailableProfiles(): NetworkProfile[] {
    return this.profiles.filter(p => p.isAvailable);
  }

  getTestResults(): Map<string, NetworkTestResult> {
    return new Map(this.testResults);
  }

  // Manual profile selection
  async selectProfile(profileId: string): Promise<boolean> {
    const profile = this.profiles.find(p => p.id === profileId);
    if (!profile) return false;

    const testResult = await this.testProfile(profile);
    if (testResult.success) {
      this.currentProfile = profile;
      this.notifyListeners(profile);
      return true;
    }
    
    return false;
  }

  // Event listeners
  addListener(callback: (profile: NetworkProfile) => void): () => void {
    this.listeners.add(callback);
    return () => this.listeners.delete(callback);
  }

  private notifyListeners(profile: NetworkProfile): void {
    this.listeners.forEach(callback => {
      try {
        callback(profile);
      } catch (error) {
        console.error('[UnifiedNetworkManager] Listener error:', error);
      }
    });
  }

  // Network type detection
  detectNetworkType(): 'wifi' | 'mobile' | 'unknown' {
    if (typeof navigator === 'undefined') return 'unknown';
    
    // Use Network Information API if available
    if ('connection' in navigator) {
      const connection = (navigator as any).connection;
      if (connection) {
        const effectiveType = connection.effectiveType;
        if (effectiveType === '4g' || effectiveType === '3g' || effectiveType === '2g') {
          return 'mobile';
        }
        return 'wifi';
      }
    }
    
    // Fallback: detect based on user agent
    const isMobile = /Android|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
    return isMobile ? 'mobile' : 'wifi';
  }

  // Get optimization settings for current profile
  getOptimizationSettings(): Record<string, any> {
    if (!this.currentProfile) return {};
    
    const settings: Record<string, any> = {
      timeout: this.currentProfile.optimization.timeout,
      maxRetries: this.currentProfile.optimization.retries,
      enableCompression: this.currentProfile.optimization.compression,
      videoQuality: this.currentProfile.optimization.videoQuality,
      updateFrequency: this.currentProfile.optimization.updateFrequency,
      networkType: this.currentProfile.type,
      profileName: this.currentProfile.name
    };

    // Add performance metrics if available
    const testResult = this.testResults.get(this.currentProfile.id);
    if (testResult) {
      settings.latency = testResult.latency;
      settings.bandwidth = testResult.bandwidth;
      settings.stability = testResult.stability;
    }

    return settings;
  }
}

// Singleton instance
export const unifiedNetworkManager = new UnifiedNetworkManager();

// Utility functions
export function getCurrentNetworkProfile(): NetworkProfile | null {
  return unifiedNetworkManager.getCurrentProfile();
}

export function getAllNetworkProfiles(): NetworkProfile[] {
  return unifiedNetworkManager.getAllProfiles();
}

export function getAvailableNetworkProfiles(): NetworkProfile[] {
  return unifiedNetworkManager.getAvailableProfiles();
}

export function getNetworkOptimizationSettings(): Record<string, any> {
  return unifiedNetworkManager.getOptimizationSettings();
}

export function forceNetworkFallback(): Promise<NetworkProfile | null> {
  return unifiedNetworkManager.forceFallback();
}

export function selectNetworkProfile(profileId: string): Promise<boolean> {
  return unifiedNetworkManager.selectProfile(profileId);
}

export function addNetworkChangeListener(callback: (profile: NetworkProfile) => void): () => void {
  return unifiedNetworkManager.addListener(callback);
}
