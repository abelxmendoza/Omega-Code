'use client';

/**
 * ServiceTable — displays all services with status, health, and action buttons.
 *
 * Data is provided by the parent (services.tsx) via props — no independent
 * polling inside this component.
 */

import React, { useState } from 'react';
import {
  Play, Square, RotateCw, CheckCircle, XCircle,
  AlertCircle, Loader2, Terminal,
} from 'lucide-react';
import { ServiceStatus } from '@/hooks/useServiceStatus';
import { robotFetch } from '@/utils/network';
import { ROBOT_ENABLED } from '@/utils/env';

interface ServiceTableProps {
  services: ServiceStatus[];
  loading: boolean;
  selectedService?: string | null;
  onServiceSelect?: (serviceName: string) => void;
  onActionComplete?: () => void;
}

const STATUS_COLOR: Record<string, string> = {
  running:  'text-emerald-400',
  stopped:  'text-white/40',
  crashed:  'text-rose-400',
  starting: 'text-amber-400',
  stopping: 'text-orange-400',
};

const POLICY_COLOR: Record<string, string> = {
  always:     'border-emerald-500/50 text-emerald-400',
  'on-failure': 'border-amber-500/50 text-amber-400',
  never:      'border-white/20 text-white/40',
};

function StatusIcon({ status }: { status: string }) {
  switch (status) {
    case 'running':  return <CheckCircle size={14} className="text-emerald-400" />;
    case 'stopped':  return <XCircle     size={14} className="text-white/30" />;
    case 'crashed':  return <AlertCircle size={14} className="text-rose-400" />;
    case 'starting':
    case 'stopping': return <Loader2     size={14} className="text-amber-400 animate-spin" />;
    default:         return <AlertCircle size={14} className="text-white/30" />;
  }
}

export function ServiceTable({
  services,
  loading,
  selectedService,
  onServiceSelect,
  onActionComplete,
}: ServiceTableProps) {
  const [actionLoading, setActionLoading] = useState<string | null>(null);

  const performAction = async (action: 'start' | 'stop' | 'restart', name: string) => {
    if (!ROBOT_ENABLED || actionLoading) return;
    setActionLoading(`${action}:${name}`);
    try {
      const res = await robotFetch(`/api/services/${action}/${name}`, { method: 'POST' });
      if ('offline' in res && res.offline) return;
      const data = await res.json();
      if (data.ok) onActionComplete?.();
    } catch (err) {
      console.error(`Failed to ${action} service ${name}:`, err);
    } finally {
      setActionLoading(null);
    }
  };

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

  return (
    <div className="overflow-x-auto">
      <table className="w-full border-collapse text-sm">
        <thead>
          <tr className="border-b border-white/8 text-left">
            <th className="px-4 py-2.5 text-xs font-semibold text-white/40 uppercase tracking-wider">Service</th>
            <th className="px-4 py-2.5 text-xs font-semibold text-white/40 uppercase tracking-wider">Status</th>
            <th className="px-4 py-2.5 text-xs font-semibold text-white/40 uppercase tracking-wider">Health</th>
            <th className="px-4 py-2.5 text-xs font-semibold text-white/40 uppercase tracking-wider">PID</th>
            <th className="px-4 py-2.5 text-xs font-semibold text-white/40 uppercase tracking-wider">Policy</th>
            <th className="px-4 py-2.5 text-xs font-semibold text-white/40 uppercase tracking-wider">Actions</th>
          </tr>
        </thead>
        <tbody>
          {services.map((svc) => {
            const isSelected  = selectedService === svc.name;
            const isActing    = actionLoading?.endsWith(`:${svc.name}`) ?? false;
            const isRunning   = svc.status === 'running';
            const policyClass = POLICY_COLOR[svc.restart_policy] ?? POLICY_COLOR.never;

            return (
              <tr
                key={svc.name}
                className={`border-b border-white/5 transition-colors ${isSelected ? 'bg-emerald-900/10' : 'hover:bg-white/3'}`}
              >
                {/* Service name */}
                <td className="px-4 py-3">
                  <div className="font-semibold text-white">{svc.display_name}</div>
                  <div className="text-xs text-white/40 mt-0.5">{svc.description}</div>
                  <div className="flex flex-wrap gap-1.5 mt-1">
                    {svc.autostart && (
                      <span className="text-[10px] px-1.5 py-0.5 rounded border border-sky-500/50 text-sky-400 font-medium">
                        autostart
                      </span>
                    )}
                    <span className="text-[10px] px-1.5 py-0.5 rounded border border-white/15 text-white/40 font-medium">
                      {svc.type}
                    </span>
                    {svc.port != null && (
                      <span className="text-[10px] px-1.5 py-0.5 rounded border border-white/15 text-white/40 font-mono">
                        :{svc.port}
                      </span>
                    )}
                  </div>
                </td>

                {/* Status */}
                <td className="px-4 py-3">
                  <div className="flex items-center gap-2">
                    <StatusIcon status={svc.status} />
                    <span className={`text-xs font-semibold uppercase ${STATUS_COLOR[svc.status] ?? 'text-white/40'}`}>
                      {svc.status}
                    </span>
                  </div>
                  {svc.crash_count > 0 && (
                    <div className="text-[10px] text-rose-400 mt-0.5">{svc.crash_count} crash{svc.crash_count !== 1 ? 'es' : ''}</div>
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
                <td className="px-4 py-3">
                  <div className="flex items-center gap-1.5 flex-wrap">
                    {isRunning ? (
                      <>
                        <ActionBtn
                          label="Stop"
                          icon={<Square size={11} />}
                          onClick={() => performAction('stop', svc.name)}
                          loading={isActing && actionLoading?.startsWith('stop:')}
                          danger
                        />
                        <ActionBtn
                          label="Restart"
                          icon={<RotateCw size={11} />}
                          onClick={() => performAction('restart', svc.name)}
                          loading={isActing && actionLoading?.startsWith('restart:')}
                        />
                      </>
                    ) : (
                      <ActionBtn
                        label="Start"
                        icon={<Play size={11} />}
                        onClick={() => performAction('start', svc.name)}
                        loading={isActing && actionLoading?.startsWith('start:')}
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
/* Action button                                                       */
/* ------------------------------------------------------------------ */

function ActionBtn({
  label,
  icon,
  onClick,
  loading = false,
  primary = false,
  danger = false,
  active = false,
}: {
  label: string;
  icon: React.ReactNode;
  onClick: () => void;
  loading?: boolean;
  primary?: boolean;
  danger?: boolean;
  active?: boolean;
}) {
  const base = 'inline-flex items-center gap-1 px-2 py-1 rounded text-[11px] font-semibold border transition-colors disabled:opacity-40';
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
      disabled={loading}
      className={`${base} ${color}`}
    >
      {loading ? <Loader2 size={11} className="animate-spin" /> : icon}
      {label}
    </button>
  );
}
