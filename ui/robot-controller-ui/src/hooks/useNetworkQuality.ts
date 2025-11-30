/*
# File: /Omega-Code/ui/robot-controller-ui/src/hooks/useNetworkQuality.ts
# Summary:
React hook that monitors WebSocket latency and determines network quality.
- Tracks latency over time to detect network conditions
- Provides 'good', 'poor', or 'unknown' quality status
- Can be used to adapt connection behavior (timeouts, retry intervals) based on network quality
- Especially useful for mobile hotspots and unstable WiFi connections
*/

import { useEffect, useState, useRef } from 'react';

export type NetworkQuality = 'good' | 'poor' | 'unknown';

interface UseNetworkQualityOptions {
  /** Minimum number of samples before determining quality */
  minSamples?: number;
  /** Maximum number of samples to keep in history */
  maxSamples?: number;
  /** Latency threshold in ms - above this is considered 'poor' */
  latencyThreshold?: number;
  /** Variance threshold - high variance indicates unstable network */
  varianceThreshold?: number;
}

/**
 * Hook to monitor network quality based on WebSocket latency
 * 
 * @param latency - Current latency value from useWsStatus hook (or null)
 * @param options - Configuration options
 * @returns Network quality status: 'good', 'poor', or 'unknown'
 * 
 * @example
 * ```tsx
 * const { latency } = useWsStatus(wsUrl);
 * const quality = useNetworkQuality(latency);
 * 
 * // Adjust behavior based on quality
 * const timeout = quality === 'poor' ? 5000 : 2500;
 * ```
 */
export function useNetworkQuality(
  latency: number | null,
  options: UseNetworkQualityOptions = {}
): NetworkQuality {
  const {
    minSamples = 5,
    maxSamples = 20,
    latencyThreshold = 500,
    varianceThreshold = 10000,
  } = options;

  const [quality, setQuality] = useState<NetworkQuality>('unknown');
  const latenciesRef = useRef<number[]>([]);

  useEffect(() => {
    if (latency === null) {
      return;
    }

    // Add new latency sample
    latenciesRef.current.push(latency);
    
    // Keep only recent samples
    if (latenciesRef.current.length > maxSamples) {
      latenciesRef.current.shift();
    }

    // Need minimum samples before determining quality
    if (latenciesRef.current.length < minSamples) {
      setQuality('unknown');
      return;
    }

    // Calculate average latency
    const avg = latenciesRef.current.reduce((a, b) => a + b, 0) / latenciesRef.current.length;
    
    // Calculate variance (measure of stability)
    const variance = latenciesRef.current.reduce(
      (sum, l) => sum + Math.pow(l - avg, 2),
      0
    ) / latenciesRef.current.length;

    // Determine quality based on average latency and variance
    if (avg > latencyThreshold || variance > varianceThreshold) {
      setQuality('poor');
    } else {
      setQuality('good');
    }
  }, [latency, minSamples, maxSamples, latencyThreshold, varianceThreshold]);

  return quality;
}

