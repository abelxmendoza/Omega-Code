'use client';

/**
 * ServiceLogs — real-time log tail for a single service.
 * Auto-scrolls, auto-refreshes, and shows stdout/stderr separately.
 */

import React, { useCallback, useEffect, useRef, useState } from 'react';
import { X, RefreshCw, Loader2, Terminal } from 'lucide-react';
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
  const [logs, setLogs]         = useState<{ stdout: string[]; stderr: string[] } | null>(null);
  const [loading, setLoading]   = useState(false);
  const [error, setError]       = useState<string | null>(null);
  const [autoScroll, setAutoScroll] = useState(true);

  const stdoutEndRef = useRef<HTMLDivElement>(null);
  const stderrEndRef = useRef<HTMLDivElement>(null);

  const fetchLogs = useCallback(async () => {
    if (!ROBOT_ENABLED) return;
    setLoading(true);
    setError(null);
    try {
      const res = await robotFetch(`/api/services/logs/${serviceName}?lines=${lines}`);
      if ('offline' in res && res.offline) { setError('Robot is offline'); return; }
      const data = await res.json();
      if (data.ok && data.logs) {
        setLogs(data.logs);
      } else {
        setError('Failed to load logs');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch logs');
    } finally {
      setLoading(false);
    }
  }, [serviceName, lines]);

  // Auto-scroll when logs update
  useEffect(() => {
    if (autoScroll && logs) {
      stdoutEndRef.current?.scrollIntoView({ behavior: 'smooth' });
      stderrEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  }, [logs, autoScroll]);

  // Initial fetch + interval
  useEffect(() => {
    fetchLogs();
    if (!autoRefresh) return;
    const id = setInterval(fetchLogs, refreshInterval);
    return () => clearInterval(id);
  }, [fetchLogs, autoRefresh, refreshInterval]);

  return (
    <div>
      {/* Header */}
      <div className="flex items-center justify-between px-4 py-2.5 bg-gray-800 border-b border-white/10">
        <div className="flex items-center gap-2">
          <Terminal size={13} className="text-emerald-400" />
          <span className="text-sm font-bold text-white">{serviceDisplayName || serviceName}</span>
          <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 text-white/40">
            {autoRefresh ? `auto ${refreshInterval / 1000}s` : 'manual'}
          </span>
        </div>
        <div className="flex items-center gap-2">
          <label className="flex items-center gap-1.5 text-xs text-white/50 cursor-pointer select-none">
            <input
              type="checkbox"
              checked={autoScroll}
              onChange={e => setAutoScroll(e.target.checked)}
              className="w-3 h-3 rounded border-white/20 bg-white/5 accent-emerald-500"
            />
            Auto-scroll
          </label>
          <button
            onClick={fetchLogs}
            disabled={loading}
            className="flex items-center gap-1 px-2 py-1 rounded text-[11px] border border-white/10 text-white/50 hover:text-white hover:bg-white/5 transition-colors disabled:opacity-40"
          >
            {loading
              ? <Loader2 size={11} className="animate-spin" />
              : <RefreshCw size={11} />
            }
            Refresh
          </button>
          {onClose && (
            <button
              onClick={onClose}
              className="p-1 rounded text-white/40 hover:text-white hover:bg-white/5 transition-colors"
              aria-label="Close logs"
            >
              <X size={13} />
            </button>
          )}
        </div>
      </div>

      <div className="p-4 space-y-3">
        {/* Error */}
        {error && (
          <div className="px-3 py-2 rounded border border-rose-500/40 bg-rose-600/10 text-rose-400 text-xs">
            {error}
          </div>
        )}

        {/* Initial loading */}
        {loading && logs === null && (
          <div className="py-8 text-center">
            <Loader2 size={20} className="animate-spin mx-auto text-white/30" />
            <p className="text-xs text-white/40 mt-2">Loading logs…</p>
          </div>
        )}

        {/* Logs */}
        {logs && (
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-3">
            <LogPane label="Stdout" lines={logs.stdout} colorClass="text-emerald-400" endRef={stdoutEndRef} />
            <LogPane label="Stderr" lines={logs.stderr} colorClass="text-rose-400"    endRef={stderrEndRef} />
          </div>
        )}
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Log pane                                                            */
/* ------------------------------------------------------------------ */

function LogPane({
  label,
  lines,
  colorClass,
  endRef,
}: {
  label: string;
  lines: string[];
  colorClass: string;
  endRef: React.RefObject<HTMLDivElement>;
}) {
  return (
    <div>
      <div className="text-[11px] font-semibold text-white/40 uppercase tracking-wider mb-1.5">{label}</div>
      <div className="bg-black/60 border border-white/8 rounded-lg p-3 font-mono text-xs max-h-64 overflow-y-auto">
        {lines.length > 0 ? (
          <>
            {lines.map((line, i) => (
              <div key={i} className={`whitespace-pre-wrap break-words leading-5 ${colorClass}`}>
                {line}
              </div>
            ))}
            <div ref={endRef} />
          </>
        ) : (
          <div className="text-white/20 italic">No {label.toLowerCase()} output</div>
        )}
      </div>
    </div>
  );
}
