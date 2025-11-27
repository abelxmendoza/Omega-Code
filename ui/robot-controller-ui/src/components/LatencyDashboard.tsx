/*
Latency Dashboard Component

Displays real-time latency metrics for Pi-only and Pi ↔ Orin round-trip measurements.
*/

'use client';

import React, { useState, useEffect, useCallback } from 'react';
import { Activity, Zap, Clock, TrendingUp, AlertCircle } from 'lucide-react';

interface PiOnlyLatency {
  ok: boolean;
  type: 'pi_only';
  timestamps_ns?: {
    capture_timestamp_ns?: number;
    encode_start_ns?: number;
    encode_end_ns?: number;
  };
  latencies_ms?: {
    capture_to_encode_ms?: number;
    encode_duration_ms?: number;
    total_processing_ms?: number;
  };
}

interface HybridLatency {
  ok: boolean;
  type: 'hybrid';
  round_trip_ms?: {
    min: number;
    max: number;
    avg: number;
    count: number;
  };
  inference_ms?: {
    min: number;
    max: number;
    avg: number;
    count: number;
  } | null;
}

type LatencyData = PiOnlyLatency | HybridLatency;

export default function LatencyDashboard() {
  const [piLatency, setPiLatency] = useState<PiOnlyLatency | null>(null);
  const [hybridLatency, setHybridLatency] = useState<HybridLatency | null>(null);
  const [error, setError] = useState<string | null>(null);

  const fetchPiLatency = useCallback(async () => {
    try {
      const response = await fetch('/api/video-proxy/latency');
      if (!response.ok) throw new Error('Failed to fetch Pi latency');
      const data: PiOnlyLatency = await response.json();
      setPiLatency(data);
      setError(null);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch Pi latency');
    }
  }, []);

  const fetchHybridLatency = useCallback(async () => {
    try {
      const response = await fetch('/api/video-proxy/latency/hybrid');
      if (!response.ok) {
        // Hybrid latency may not be available if Orin is not connected
        if (response.status === 503) {
          setHybridLatency(null);
          return;
        }
        throw new Error('Failed to fetch hybrid latency');
      }
      const data: HybridLatency = await response.json();
      setHybridLatency(data);
    } catch (err) {
      // Silently fail for hybrid latency (may not be available)
      setHybridLatency(null);
    }
  }, []);

  useEffect(() => {
    fetchPiLatency();
    fetchHybridLatency();
    
    // Poll every 500ms for real-time updates
    const interval = setInterval(() => {
      fetchPiLatency();
      fetchHybridLatency();
    }, 500);
    
    return () => clearInterval(interval);
  }, [fetchPiLatency, fetchHybridLatency]);

  const getLatencyColor = (ms: number, threshold: number = 100) => {
    if (ms < threshold * 0.5) return 'text-green-500';
    if (ms < threshold) return 'text-yellow-500';
    return 'text-red-500';
  };

  return (
    <div className="bg-gray-900 rounded-lg p-6 shadow-lg">
      <h2 className="text-2xl font-bold text-white mb-4 flex items-center gap-2">
        <Activity className="w-6 h-6" />
        Latency Metrics
      </h2>

      {/* Error Display */}
      {error && (
        <div className="mb-4 p-3 bg-red-900/50 border border-red-500 rounded-lg flex items-center gap-2">
          <AlertCircle className="w-5 h-5 text-red-500" />
          <span className="text-red-200 text-sm">{error}</span>
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {/* Pi-Only Latency */}
        <div className="bg-gray-800 rounded-lg p-4">
          <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
            <Zap className="w-5 h-5 text-blue-500" />
            Pi-Only Latency
          </h3>
          
          {piLatency?.ok && piLatency.latencies_ms ? (
            <div className="space-y-2">
              {piLatency.latencies_ms.capture_to_encode_ms !== undefined && (
                <div className="flex justify-between items-center">
                  <span className="text-gray-400 text-sm">Capture → Encode:</span>
                  <span className={`font-semibold ${getLatencyColor(piLatency.latencies_ms.capture_to_encode_ms, 50)}`}>
                    {piLatency.latencies_ms.capture_to_encode_ms.toFixed(2)} ms
                  </span>
                </div>
              )}
              
              {piLatency.latencies_ms.encode_duration_ms !== undefined && (
                <div className="flex justify-between items-center">
                  <span className="text-gray-400 text-sm">Encode Duration:</span>
                  <span className={`font-semibold ${getLatencyColor(piLatency.latencies_ms.encode_duration_ms, 100)}`}>
                    {piLatency.latencies_ms.encode_duration_ms.toFixed(2)} ms
                  </span>
                </div>
              )}
              
              {piLatency.latencies_ms.total_processing_ms !== undefined && (
                <div className="flex justify-between items-center pt-2 border-t border-gray-700">
                  <span className="text-gray-300 font-semibold">Total Processing:</span>
                  <span className={`font-bold text-lg ${getLatencyColor(piLatency.latencies_ms.total_processing_ms, 150)}`}>
                    {piLatency.latencies_ms.total_processing_ms.toFixed(2)} ms
                  </span>
                </div>
              )}
            </div>
          ) : (
            <div className="text-gray-500 text-sm">No latency data available</div>
          )}
        </div>

        {/* Hybrid Round-Trip Latency */}
        <div className="bg-gray-800 rounded-lg p-4">
          <h3 className="text-lg font-semibold text-white mb-3 flex items-center gap-2">
            <Clock className="w-5 h-5 text-purple-500" />
            Pi ↔ Orin Round-Trip
          </h3>
          
          {hybridLatency?.ok && hybridLatency.round_trip_ms ? (
            <div className="space-y-2">
              <div className="flex justify-between items-center">
                <span className="text-gray-400 text-sm">Average:</span>
                <span className={`font-semibold text-lg ${getLatencyColor(hybridLatency.round_trip_ms.avg, 200)}`}>
                  {hybridLatency.round_trip_ms.avg.toFixed(2)} ms
                </span>
              </div>
              
              <div className="flex justify-between items-center">
                <span className="text-gray-400 text-sm">Min:</span>
                <span className="text-green-400 font-semibold">
                  {hybridLatency.round_trip_ms.min.toFixed(2)} ms
                </span>
              </div>
              
              <div className="flex justify-between items-center">
                <span className="text-gray-400 text-sm">Max:</span>
                <span className="text-red-400 font-semibold">
                  {hybridLatency.round_trip_ms.max.toFixed(2)} ms
                </span>
              </div>
              
              <div className="flex justify-between items-center pt-2 border-t border-gray-700">
                <span className="text-gray-400 text-sm">Samples:</span>
                <span className="text-gray-300 font-semibold">
                  {hybridLatency.round_trip_ms.count}
                </span>
              </div>
              
              {hybridLatency.inference_ms && (
                <div className="pt-2 border-t border-gray-700 mt-2">
                  <div className="text-gray-400 text-xs mb-1">Inference Time:</div>
                  <div className="flex justify-between items-center">
                    <span className="text-gray-400 text-sm">Avg:</span>
                    <span className={`font-semibold ${getLatencyColor(hybridLatency.inference_ms.avg, 100)}`}>
                      {hybridLatency.inference_ms.avg.toFixed(2)} ms
                    </span>
                  </div>
                </div>
              )}
            </div>
          ) : (
            <div className="text-gray-500 text-sm">
              {hybridLatency === null 
                ? "Hybrid system not available (Orin not connected)"
                : "No round-trip latency data yet"}
            </div>
          )}
        </div>
      </div>

      {/* Real-time indicator */}
      <div className="mt-4 flex items-center gap-2 text-gray-400 text-xs">
        <div className="w-2 h-2 bg-green-500 rounded-full animate-pulse"></div>
        <span>Updating every 500ms</span>
      </div>
    </div>
  );
}

