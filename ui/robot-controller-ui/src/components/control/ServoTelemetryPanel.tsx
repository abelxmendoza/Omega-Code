/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/control/ServoTelemetryPanel.tsx
# Summary:
Live pan/tilt telemetry card that displays the latest servo angles and operating
range next to the camera controls. Pulls data from CommandContext (WebSocket
acks + status snapshots) and offers a manual refresh button that re-issues the
`status` command when available.
*/

import React from 'react';
import { useCommand } from '@/context/CommandContext';
import { statusColor } from '@/constants/status';

type Source = 'ack' | 'status' | null;

const sourceLabel: Record<Exclude<Source, null>, string> = {
  ack: 'Ack',
  status: 'Status',
};

const formatAngle = (value: number | null): string =>
  value == null ? '—' : `${Math.round(value)}°`;

const formatRange = (min: number | null, max: number | null): string => {
  if (min == null || max == null) return '—';
  return `${Math.round(min)}° – ${Math.round(max)}°`;
};

const statusLabel = (status: 'connecting' | 'connected' | 'disconnected') =>
  status === 'connected' ? 'Live' : status === 'connecting' ? 'Connecting' : 'Offline';

export default function ServoTelemetryPanel() {
  const { servoTelemetry, status, requestStatus } = useCommand();
  const { horizontal, vertical, min, max, updatedAt, source } = servoTelemetry;

  const updatedText = React.useMemo(() => {
    if (!updatedAt) return 'Waiting for telemetry…';
    const time = new Date(updatedAt).toLocaleTimeString();
    const via = source ? ` via ${sourceLabel[source]}` : '';
    return `Updated ${time}${via}`;
  }, [updatedAt, source]);

  const canRefresh = status === 'connected';

  const handleRefresh = React.useCallback(() => {
    if (!canRefresh) return;
    requestStatus('servo refresh');
  }, [canRefresh, requestStatus]);

  return (
    <div
      className="mt-4 w-60 rounded-xl border border-white/10 bg-zinc-900/75 px-4 py-3 text-sm text-zinc-100 shadow"
      aria-live="polite"
    >
      <div className="flex items-center justify-between text-xs font-semibold uppercase tracking-wide text-zinc-400">
        <span>Servo Telemetry</span>
        <span className="flex items-center gap-1 text-[0.7rem] normal-case text-zinc-300">
          <span
            className={`h-2.5 w-2.5 rounded-full ${statusColor[status]}`}
            aria-hidden
          />
          {statusLabel(status)}
        </span>
      </div>

      <dl className="mt-3 space-y-2">
        <div className="flex items-center justify-between">
          <dt className="text-zinc-400">Pan</dt>
          <dd className="font-semibold text-base text-white">{formatAngle(horizontal)}</dd>
        </div>
        <div className="flex items-center justify-between">
          <dt className="text-zinc-400">Tilt</dt>
          <dd className="font-semibold text-base text-white">{formatAngle(vertical)}</dd>
        </div>
        <div className="flex items-center justify-between text-xs text-zinc-400">
          <dt>Range</dt>
          <dd className="font-medium text-zinc-200">{formatRange(min, max)}</dd>
        </div>
      </dl>

      <div className="mt-3 flex items-center justify-between text-[0.7rem] text-zinc-400">
        <span>{updatedText}</span>
        <button
          type="button"
          onClick={handleRefresh}
          disabled={!canRefresh}
          className={`rounded-md px-2 py-1 font-medium transition-colors disabled:cursor-not-allowed disabled:opacity-50 ${
            canRefresh ? 'bg-emerald-600 text-white hover:bg-emerald-500' : 'bg-zinc-700 text-zinc-300'
          }`}
        >
          Refresh
        </button>
      </div>
    </div>
  );
}
