'use client';

/**
 * ServiceTable
 *
 * Hardened per blueprint:
 * - Per-row action loading state (spinner + button lock)
 * - Per-row inline action result feedback (success / error, auto-clears 3s)
 * - Optimistic status transitions: stop → "stopping", start → "starting",
 *   restart → "stopping" immediately on click, no waiting for next poll
 * - Receives data as props — no internal polling
 */

import React, { useCallback, useEffect, useRef, useState } from 'react';
import {
  Play, Square, RotateCw, CheckCircle, XCircle,
  AlertCircle, Loader2, Terminal, CheckCircle2,
} from 'lucide-react';
import { ServiceStatus } from '@/hooks/useServiceStatus';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

interface ServiceTableProps {
  services: ServiceStatus[];
  loading: boolean;
  selectedService?: string | null;
  onServiceSelect?: (serviceName: string) => void;
  onActionComplete?: () => void;
}

interface ActionResult {
  ok: boolean;
  msg: string;
}

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

const STATUS_COLOR: Record<string, string> = {
  running:  'text-emerald-400',
  stopped:  'text-white/40',
  crashed:  'text-rose-400',
  starting: 'text-amber-400',
  stopping: 'text-orange-400',
};

const POLICY_COLOR: Record<string, string> = {
  always:       'border-emerald-500/50 text-emerald-400',
  'on-failure': 'border-amber-500/50 text-amber-400',
  never:        'border-white/20 text-white/40',
};

// How long (ms) an action result stays visible before auto-clearing
const RESULT_TTL_MS = 3000;

/* ------------------------------------------------------------------ */
/* Status icon                                                         */
/* ------------------------------------------------------------------ */

function StatusIcon({ status }: { status: string }) {
  switch (status) {
    case 'running':  return <CheckCircle size={14} className="text-emerald-400 shrink-0" />;
    case 'stopped':  return <XCircle     size={14} className="text-white/30 shrink-0" />;
    case 'crashed':  return <AlertCircle size={14} className="text-rose-400 shrink-0" />;
    case 'starting':
    case 'stopping': return <Loader2     size={14} className="text-amber-400 animate-spin shrink-0" />;
    default:         return <AlertCircle size={14} className="text-white/30 shrink-0" />;
  }
}

/* ------------------------------------------------------------------ */
/* Component                                                           */
/* ------------------------------------------------------------------ */

export function ServiceTable({
  services,
  loading,
  selectedService,
  onServiceSelect,
  onActionComplete,
}: ServiceTableProps) {
  // Which action is in-flight: "start:video_server" etc.
  const [actionLoading, setActionLoading] = useState<string | null>(null);

  // Per-row result feedback: { video_server: { ok, msg } }
  const [actionResults, setActionResults] = useState<Record<string, ActionResult>>({});

  // Optimistic overrides: { video_server: "stopping" }
  const [optimisticStatus, setOptimisticStatus] = useState<Record<string, string>>({});

  // Auto-clear timers
  const clearTimers = useRef<Record<string, ReturnType<typeof setTimeout>>>({});

  // Clear optimistic status for a service once real data arrives for it
  useEffect(() => {
    if (services.length === 0) return;
    setOptimisticStatus(prev => {
      const updated = { ...prev };
      services.forEach(s => { delete updated[s.name]; });
      return updated;
    });
  }, [services]);

  const setResult = useCallback((name: string, result: ActionResult) => {
    setActionResults(prev => ({ ...prev, [name]: result }));
    // Auto-clear after TTL
    if (clearTimers.current[name]) clearTimeout(clearTimers.current[name]);
    clearTimers.current[name] = setTimeout(() => {
      setActionResults(prev => {
        const next = { ...prev };
        delete next[name];
        return next;
      });
    }, RESULT_TTL_MS);
  }, []);

  const performAction = useCallback(async (
    action: 'start' | 'stop' | 'restart',
    name: string,
  ) => {
    if (!ROBOT_ENABLED || actionLoading) return;

    const key = `${action}:${name}`;
    setActionLoading(key);

    // Optimistic transition — instant UI feedback
    const optimisticMap: Record<typeof action, string> = {
      start:   'starting',
      stop:    'stopping',
      restart: 'stopping',
    };
    setOptimisticStatus(prev => ({ ...prev, [name]: optimisticMap[action] }));

    try {
      const res = await robotFetch(`/api/services/${action}/${name}`, { method: 'POST' });

      if ('offline' in res && res.offline) {
        setResult(name, { ok: false, msg: 'Robot is offline' });
        setOptimisticStatus(prev => { const n = { ...prev }; delete n[name]; return n; });
        return;
      }

      const data = await res.json();

      if (data.ok) {
        const labels: Record<typeof action, string> = {
          start:   'started',
          stop:    'stopped',
          restart: 'restarted',
        };
        setResult(name, { ok: true, msg: `Service ${labels[action]}` });
        onActionComplete?.();
      } else {
        const errMsg = data.detail ?? data.error ?? `Failed to ${action} service`;
        setResult(name, { ok: false, msg: errMsg });
        setOptimisticStatus(prev => { const n = { ...prev }; delete n[name]; return n; });
      }
    } catch (err) {
      const msg = err instanceof Error ? err.message : `Failed to ${action} service`;
      setResult(name, { ok: false, msg });
      setOptimisticStatus(prev => { const n = { ...prev }; delete n[name]; return n; });
    } finally {
      setActionLoading(null);
    }
  }, [actionLoading, onActionComplete, setResult]);

  /* ---- Loading / empty states ------------------------------------ */

  if (!ROBOT_ENABLED) {
    return (
      <div className="px-4 py-8 text-center text-sm text-white/40">
        Robot is offline — service management unavailable.
      </div>
    );
  }

  if (loading && services.length === 0) {
    return (
      <div className="px-4 py-10 text-center">
        <Loader2 size={24} className="animate-spin mx-auto text-white/30" />
        <p className="text-sm text-white/40 mt-2">Loading services…</p>
      </div>
    );
  }

  if (services.length === 0) {
    return (
      <div className="px-4 py-8 text-center text-sm text-white/40">
        No services found.
      </div>
    );
  }

  /* ---- Table ----------------------------------------------------- */

  return (
    <div className="overflow-x-auto">
      <table className="w-full border-collapse text-sm">
        <thead>
          <tr className="border-b border-white/8 text-left">
            {['Service', 'Status', 'Health', 'PID', 'Policy', 'Actions'].map(h => (
              <th key={h} className="px-4 py-2.5 text-xs font-semibold text-white/40 uppercase tracking-wider">
                {h}
              </th>
            ))}
          </tr>
        </thead>
        <tbody>
          {services.map(svc => {
            const isSelected   = selectedService === svc.name;
            const isActing     = actionLoading?.endsWith(`:${svc.name}`) ?? false;
            const actingAction = isActing ? (actionLoading?.split(':')[0] as 'start'|'stop'|'restart') : null;
            const displayStatus = optimisticStatus[svc.name] ?? svc.status;
            const isRunning    = displayStatus === 'running';
            const isTransient  = displayStatus === 'starting' || displayStatus === 'stopping';
            const policyClass  = POLICY_COLOR[svc.restart_policy] ?? POLICY_COLOR.never;
            const result       = actionResults[svc.name];

            return (
              <tr
                key={svc.name}
                className={`border-b border-white/5 transition-colors ${
                  isSelected ? 'bg-emerald-900/10' : 'hover:bg-white/[0.02]'
                }`}
              >
                {/* Service */}
                <td className="px-4 py-3 min-w-[180px]">
                  <div className="font-semibold text-white">{svc.display_name}</div>
                  <div className="text-xs text-white/40 mt-0.5">{svc.description}</div>
                  <div className="flex flex-wrap gap-1.5 mt-1">
                    {svc.autostart && (
                      <Chip label="autostart" color="border-sky-500/50 text-sky-400" />
                    )}
                    <Chip label={svc.type} />
                    {svc.port != null && (
                      <Chip label={`:${svc.port}`} mono />
                    )}
                  </div>
                </td>

                {/* Status */}
                <td className="px-4 py-3 min-w-[110px]">
                  <div className="flex items-center gap-2">
                    <StatusIcon status={displayStatus} />
                    <span className={`text-xs font-semibold uppercase ${STATUS_COLOR[displayStatus] ?? 'text-white/40'}`}>
                      {displayStatus}
                    </span>
                  </div>
                  {svc.crash_count > 0 && (
                    <div className="text-[10px] text-rose-400 mt-0.5">
                      {svc.crash_count} crash{svc.crash_count !== 1 ? 'es' : ''}
                    </div>
                  )}
                </td>

                {/* Health */}
                <td className="px-4 py-3">
                  {svc.health ? (
                    <span className={`text-[11px] font-semibold px-2 py-0.5 rounded border ${
                      svc.health.healthy
                        ? 'border-emerald-500/50 text-emerald-400'
                        : 'border-rose-500/50 text-rose-400'
                    }`}>
                      {svc.health.healthy ? 'Healthy' : 'Unhealthy'}
                    </span>
                  ) : (
                    <span className="text-xs text-white/20">—</span>
                  )}
                </td>

                {/* PID */}
                <td className="px-4 py-3">
                  {svc.pid != null ? (
                    <span className="text-xs font-mono text-white/60">{svc.pid}</span>
                  ) : (
                    <span className="text-xs text-white/20">—</span>
                  )}
                </td>

                {/* Policy */}
                <td className="px-4 py-3">
                  <span className={`text-[11px] font-semibold px-2 py-0.5 rounded border ${policyClass}`}>
                    {svc.restart_policy}
                  </span>
                </td>

                {/* Actions */}
                <td className="px-4 py-3 min-w-[180px]">
                  <div className="flex items-center gap-1.5 flex-wrap">
                    {isRunning || isTransient ? (
                      <>
                        <ActionBtn
                          label="Stop"
                          icon={<Square size={11} />}
                          onClick={() => performAction('stop', svc.name)}
                          loading={actingAction === 'stop'}
                          disabled={isActing || isTransient}
                          danger
                        />
                        <ActionBtn
                          label="Restart"
                          icon={<RotateCw size={11} />}
                          onClick={() => performAction('restart', svc.name)}
                          loading={actingAction === 'restart'}
                          disabled={isActing || isTransient}
                        />
                      </>
                    ) : (
                      <ActionBtn
                        label="Start"
                        icon={<Play size={11} />}
                        onClick={() => performAction('start', svc.name)}
                        loading={actingAction === 'start'}
                        disabled={isActing}
                        primary
                      />
                    )}
                    {onServiceSelect && (
                      <ActionBtn
                        label="Logs"
                        icon={<Terminal size={11} />}
                        onClick={() => onServiceSelect(isSelected ? '' : svc.name)}
                        active={isSelected}
                      />
                    )}
                  </div>

                  {/* Inline action result */}
                  {result && (
                    <div className={`flex items-center gap-1 mt-1.5 text-[10px] font-semibold ${
                      result.ok ? 'text-emerald-400' : 'text-rose-400'
                    }`}>
                      {result.ok
                        ? <CheckCircle2 size={10} />
                        : <AlertCircle  size={10} />
                      }
                      {result.msg}
                    </div>
                  )}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Sub-components                                                      */
/* ------------------------------------------------------------------ */

function Chip({
  label,
  color = 'border-white/15 text-white/40',
  mono = false,
}: {
  label: string;
  color?: string;
  mono?: boolean;
}) {
  return (
    <span className={`text-[10px] px-1.5 py-0.5 rounded border font-medium ${color} ${mono ? 'font-mono' : ''}`}>
      {label}
    </span>
  );
}

function ActionBtn({
  label,
  icon,
  onClick,
  loading = false,
  disabled = false,
  primary = false,
  danger = false,
  active = false,
}: {
  label: string;
  icon: React.ReactNode;
  onClick: () => void;
  loading?: boolean;
  disabled?: boolean;
  primary?: boolean;
  danger?: boolean;
  active?: boolean;
}) {
  const base = 'inline-flex items-center gap-1 px-2 py-1 rounded text-[11px] font-semibold border transition-colors disabled:opacity-40 disabled:cursor-not-allowed';
  const color = active
    ? 'bg-emerald-600/20 border-emerald-500/50 text-emerald-300'
    : primary
    ? 'bg-emerald-600/10 border-emerald-500/40 text-emerald-400 hover:bg-emerald-600/20'
    : danger
    ? 'bg-rose-600/10 border-rose-500/40 text-rose-400 hover:bg-rose-600/20'
    : 'bg-white/5 border-white/10 text-white/60 hover:bg-white/10 hover:text-white';

  return (
    <button
      onClick={onClick}
      disabled={disabled || loading}
      className={`${base} ${color}`}
    >
      {loading ? <Loader2 size={11} className="animate-spin" /> : icon}
      {label}
    </button>
  );
}
