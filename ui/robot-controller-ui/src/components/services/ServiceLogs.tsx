/**
 * ServiceLogs Component
 * 
 * Real-time log viewer for a specific service.
 * Auto-scrolls with tailing behavior.
 */

import React, { useEffect, useRef, useState } from 'react';
import { X, RefreshCw, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

interface ServiceLogsProps {
  serviceName: string;
  serviceDisplayName?: string;
  onClose?: () => void;
  lines?: number;
  autoRefresh?: boolean;
  refreshInterval?: number;
}

export function ServiceLogs({
  serviceName,
  serviceDisplayName,
  onClose,
  lines = 100,
  autoRefresh = true,
  refreshInterval = 10000,
}: ServiceLogsProps) {
  const [logs, setLogs] = useState<{ stdout: string[]; stderr: string[] } | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const stdoutEndRef = useRef<HTMLDivElement>(null);
  const stderrEndRef = useRef<HTMLDivElement>(null);
  const [autoScroll, setAutoScroll] = useState(true);

  const fetchLogs = async () => {
    if (!ROBOT_ENABLED) return;

    setLoading(true);
    setError(null);

    try {
      const response = await robotFetch(`/api/services/logs/${serviceName}?lines=${lines}`);
      
      if (response.offline) {
        setError('Robot is offline');
        return;
      }

      const data = await response.json();
      
      if (data.ok && data.logs) {
        setLogs(data.logs);
        setError(null);
      } else {
        setError('Failed to load logs');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch logs');
      console.error('Failed to fetch logs:', err);
    } finally {
      setLoading(false);
    }
  };

  // Auto-scroll to bottom when logs update
  useEffect(() => {
    if (autoScroll && logs) {
      stdoutEndRef.current?.scrollIntoView({ behavior: 'smooth' });
      stderrEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs, autoScroll]);

  // Initial fetch and auto-refresh
  useEffect(() => {
    fetchLogs();

    if (!autoRefresh) return;

    const intervalId = setInterval(fetchLogs, refreshInterval);
    return () => clearInterval(intervalId);
  }, [serviceName, autoRefresh, refreshInterval]);

  return (
    <div className="bg-gray-800 border border-gray-700 rounded-lg p-4 space-y-4">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div>
          <h3 className="text-lg font-semibold text-white">
            Logs: {serviceDisplayName || serviceName}
          </h3>
          <p className="text-xs text-gray-400 mt-1">
            {autoRefresh ? `Auto-refreshing every ${refreshInterval / 1000}s` : 'Manual refresh'}
          </p>
        </div>
        <div className="flex gap-2">
          <Button
            size="sm"
            variant="outline"
            onClick={fetchLogs}
            disabled={loading}
            className="border-gray-600 text-gray-300 hover:bg-gray-700"
          >
            {loading ? (
              <Loader2 className="w-4 h-4 animate-spin" />
            ) : (
              <RefreshCw className="w-4 h-4" />
            )}
          </Button>
          {onClose && (
            <Button
              size="sm"
              variant="ghost"
              onClick={onClose}
              className="text-gray-400 hover:text-white"
            >
              <X className="w-4 h-4" />
            </Button>
          )}
        </div>
      </div>

      {/* Auto-scroll toggle */}
      <div className="flex items-center gap-2">
        <input
          type="checkbox"
          id="auto-scroll"
          checked={autoScroll}
          onChange={(e) => setAutoScroll(e.target.checked)}
          className="w-4 h-4 rounded border-gray-600 bg-gray-700 text-green-500"
        />
        <label htmlFor="auto-scroll" className="text-sm text-gray-300">
          Auto-scroll
        </label>
      </div>

      {/* Error state */}
      {error && (
        <div className="bg-red-900/20 border border-red-500/50 rounded p-3 text-red-400 text-sm">
          {error}
        </div>
      )}

      {/* Logs */}
      {logs && (
        <div className="space-y-4">
          {/* Stdout */}
          <div>
            <h4 className="text-sm font-semibold text-gray-300 mb-2">Stdout</h4>
            <div className="bg-black rounded p-4 font-mono text-xs text-green-400 max-h-64 overflow-y-auto">
              {logs.stdout.length > 0 ? (
                <>
                  {logs.stdout.map((line, i) => (
                    <div key={i} className="whitespace-pre-wrap break-words">
                      {line}
                    </div>
                  ))}
                  <div ref={stdoutEndRef} />
                </>
              ) : (
                <div className="text-gray-500">No stdout logs</div>
              )}
            </div>
          </div>

          {/* Stderr */}
          <div>
            <h4 className="text-sm font-semibold text-gray-300 mb-2">Stderr</h4>
            <div className="bg-black rounded p-4 font-mono text-xs text-red-400 max-h-64 overflow-y-auto">
              {logs.stderr.length > 0 ? (
                <>
                  {logs.stderr.map((line, i) => (
                    <div key={i} className="whitespace-pre-wrap break-words">
                      {line}
                    </div>
                  ))}
                  <div ref={stderrEndRef} />
                </>
              ) : (
                <div className="text-gray-500">No stderr logs</div>
              )}
            </div>
          </div>
        </div>
      )}

      {/* Loading state */}
      {loading && logs === null && (
        <div className="text-center py-8">
          <Loader2 className="w-8 h-8 animate-spin mx-auto text-gray-400" />
          <p className="text-gray-400 mt-2">Loading logs...</p>
        </div>
      )}
    </div>
  );
}

