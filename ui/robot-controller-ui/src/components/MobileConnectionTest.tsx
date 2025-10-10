/**
 * Mobile Connection Test Component
 * 
 * This component provides a UI for testing mobile hotspot connections
 * and displaying optimization recommendations.
 */

'use client';

import React, { useState, useEffect } from 'react';
import { testMobileConnection, quickConnectionTest, testWebSocketEndpoints, ConnectionTestResult } from '@/utils/mobileConnectionTest';
import { resolveWsCandidates } from '@/utils/resolveWsUrl';

interface MobileConnectionTestProps {
  onTestComplete?: (result: ConnectionTestResult) => void;
  className?: string;
}

export default function MobileConnectionTest({ onTestComplete, className = '' }: MobileConnectionTestProps) {
  const [isRunning, setIsRunning] = useState(false);
  const [result, setResult] = useState<ConnectionTestResult | null>(null);
  const [quickResult, setQuickResult] = useState<{ success: boolean; latency: number; error?: string } | null>(null);
  const [wsResults, setWsResults] = useState<Record<string, boolean>>({});
  const [showDetails, setShowDetails] = useState(false);

  const runQuickTest = async () => {
    setIsRunning(true);
    try {
      const quickTestResult = await quickConnectionTest();
      setQuickResult(quickTestResult);
    } catch (error) {
      console.error('Quick test failed:', error);
    } finally {
      setIsRunning(false);
    }
  };

  const runFullTest = async () => {
    setIsRunning(true);
    try {
      // Test WebSocket endpoints
      const wsUrls = [
        ...resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT', { defaultPort: '8081' }),
        ...resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC', { defaultPort: '8080' }),
        ...resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER', { defaultPort: '8090' }),
        ...resolveWsCandidates('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING', { defaultPort: '8082' }),
      ];
      
      const wsTestResults = await testWebSocketEndpoints(wsUrls);
      setWsResults(wsTestResults);

      // Run full connection test
      const testResult = await testMobileConnection({
        timeout: 10000,
        testDuration: 30000,
        testBandwidth: true,
        testStability: true,
        verbose: true
      });
      
      setResult(testResult);
      onTestComplete?.(testResult);
    } catch (error) {
      console.error('Full test failed:', error);
    } finally {
      setIsRunning(false);
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'excellent': return 'text-green-400';
      case 'good': return 'text-blue-400';
      case 'fair': return 'text-yellow-400';
      case 'poor': return 'text-red-400';
      default: return 'text-gray-400';
    }
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'excellent': return '🟢';
      case 'good': return '🔵';
      case 'fair': return '🟡';
      case 'poor': return '🔴';
      default: return '⚪';
    }
  };

  return (
    <div className={`bg-gray-900 border border-white/10 rounded-lg p-4 ${className}`}>
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold text-white">Mobile Connection Test</h3>
        <div className="flex gap-2">
          <button
            onClick={runQuickTest}
            disabled={isRunning}
            className="px-3 py-1 text-xs bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 text-white rounded transition"
          >
            Quick Test
          </button>
          <button
            onClick={runFullTest}
            disabled={isRunning}
            className="px-3 py-1 text-xs bg-green-600 hover:bg-green-700 disabled:bg-gray-600 text-white rounded transition"
          >
            Full Test
          </button>
        </div>
      </div>

      {isRunning && (
        <div className="mb-4 p-3 bg-blue-900/20 border border-blue-500/30 rounded">
          <div className="flex items-center gap-2 text-blue-300">
            <div className="animate-spin w-4 h-4 border-2 border-blue-300 border-t-transparent rounded-full" />
            <span className="text-sm">Testing connection...</span>
          </div>
        </div>
      )}

      {/* Quick Test Results */}
      {quickResult && (
        <div className="mb-4 p-3 bg-gray-800 border border-white/10 rounded">
          <div className="flex items-center gap-2 mb-2">
            <span className="text-sm font-medium text-white">Quick Test Results</span>
            <div className={`w-2 h-2 rounded-full ${quickResult.success ? 'bg-green-500' : 'bg-red-500'}`} />
          </div>
          <div className="text-sm text-gray-300">
            {quickResult.success ? (
              <span>✅ Connected - {Math.round(quickResult.latency)}ms latency</span>
            ) : (
              <span>❌ Failed - {quickResult.error}</span>
            )}
          </div>
        </div>
      )}

      {/* Full Test Results */}
      {result && (
        <div className="mb-4 p-3 bg-gray-800 border border-white/10 rounded">
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-2">
              <span className="text-sm font-medium text-white">Full Test Results</span>
              <span className="text-lg">{getStatusIcon(result.success ? 'good' : 'poor')}</span>
            </div>
            <button
              onClick={() => setShowDetails(!showDetails)}
              className="text-xs text-blue-400 hover:text-blue-300"
            >
              {showDetails ? 'Hide' : 'Show'} Details
            </button>
          </div>

          <div className="grid grid-cols-2 gap-4 text-sm">
            <div>
              <span className="text-gray-400">Latency:</span>
              <span className="ml-2 text-white">{Math.round(result.latency)}ms</span>
            </div>
            <div>
              <span className="text-gray-400">Bandwidth:</span>
              <span className="ml-2 text-white">
                {result.bandwidth > 0 ? `${(result.bandwidth / 1000).toFixed(0)}KB/s` : 'N/A'}
              </span>
            </div>
            <div>
              <span className="text-gray-400">Stability:</span>
              <span className="ml-2 text-white">{result.stability}%</span>
            </div>
            <div>
              <span className="text-gray-400">Duration:</span>
              <span className="ml-2 text-white">{(result.testDuration / 1000).toFixed(1)}s</span>
            </div>
          </div>

          {showDetails && (
            <div className="mt-3 pt-3 border-t border-white/10">
              {/* WebSocket Test Results */}
              {Object.keys(wsResults).length > 0 && (
                <div className="mb-3">
                  <div className="text-xs text-gray-400 mb-2">WebSocket Endpoints:</div>
                  <div className="space-y-1">
                    {Object.entries(wsResults).map(([url, success]) => (
                      <div key={url} className="flex items-center gap-2 text-xs">
                        <div className={`w-2 h-2 rounded-full ${success ? 'bg-green-500' : 'bg-red-500'}`} />
                        <span className="text-gray-300 truncate">{url}</span>
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Recommendations */}
              {result.recommendations.length > 0 && (
                <div>
                  <div className="text-xs text-gray-400 mb-2">Recommendations:</div>
                  <ul className="space-y-1">
                    {result.recommendations.map((rec, index) => (
                      <li key={index} className="text-xs text-amber-300 flex items-start gap-2">
                        <span>💡</span>
                        <span>{rec}</span>
                      </li>
                    ))}
                  </ul>
                </div>
              )}

              {/* Errors */}
              {result.errors.length > 0 && (
                <div className="mt-3">
                  <div className="text-xs text-gray-400 mb-2">Errors:</div>
                  <ul className="space-y-1">
                    {result.errors.map((error, index) => (
                      <li key={index} className="text-xs text-red-300 flex items-start gap-2">
                        <span>❌</span>
                        <span>{error}</span>
                      </li>
                    ))}
                  </ul>
                </div>
              )}
            </div>
          )}
        </div>
      )}

      {/* Instructions */}
      <div className="text-xs text-gray-400">
        <div className="mb-2">
          <strong>Quick Test:</strong> Tests basic connectivity and latency (5 seconds)
        </div>
        <div className="mb-2">
          <strong>Full Test:</strong> Comprehensive test including bandwidth, stability, and WebSocket endpoints (30 seconds)
        </div>
        <div>
          <strong>Tip:</strong> Run these tests when switching to mobile hotspot to optimize performance
        </div>
      </div>
    </div>
  );
}
