'use client';

/**
 * SystemStatusPanel — READ-ONLY telemetry / diagnostic view.
 *
 * This component has NO control buttons and makes NO POST calls.
 * VisionModePanel is the single control surface for vision mode.
 *
 * Displays:
 *   - Current active mode (name + integer)
 *   - Backend mode description
 *   - Manual override indicator
 *   - Hybrid system mode (Pi-only vs Pi+Orin)
 *   - Orin availability
 *   - Pi CPU temperature
 *   - CPU load %
 *   - Throttling warning
 *
 * All state comes from useSystemMode() — no independent polling here.
 */

import React from 'react';
import { Activity, Cpu, Thermometer, AlertTriangle, Radio, Shield } from 'lucide-react';
import { useSystemMode } from '@/hooks/useSystemMode';

/* ------------------------------------------------------------------ */
/* Mode metadata (display only — no mode IDs used for control)         */
/* ------------------------------------------------------------------ */

const MODE_LABEL: Record<number, string> = {
  0: 'Raw Stream',
  1: 'Motion Detection',
  2: 'Object Tracking',
  3: 'Face Detection',
  4: 'ArUco Markers',
  5: 'Pi Recording',
  6: 'YOLOv8 (Orin)',
  7: 'Full Autonomy (Orin)',
};

const MODE_COLOR: Record<number, string> = {
  0: 'bg-slate-700 text-slate-200',
  1: 'bg-sky-800 text-sky-200',
  2: 'bg-teal-800 text-teal-200',
  3: 'bg-violet-800 text-violet-200',
  4: 'bg-amber-800 text-amber-200',
  5: 'bg-rose-800 text-rose-200',
  6: 'bg-indigo-800 text-indigo-200',
  7: 'bg-fuchsia-800 text-fuchsia-200',
};

/* ------------------------------------------------------------------ */
/* Sub-components                                                      */
/* ------------------------------------------------------------------ */

function Stat({ icon, label, value, colorClass }: {
  icon: React.ReactNode;
  label: string;
  value: string;
  colorClass: string;
}) {
  return (
    <div className="flex items-center gap-2">
      <span className="text-white/40">{icon}</span>
      <span className="text-xs text-white/50">{label}</span>
      <span className={`text-xs font-semibold ${colorClass}`}>{value}</span>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Component                                                           */
/* ------------------------------------------------------------------ */

export default function SystemStatusPanel() {
  const {
    mode,
    description,
    manualOverride,
    orinAvailable,
    hybridMode,
    thermalTemp,
    cpuLoad,
    throttling,
    offline,
  } = useSystemMode();

  const modeLabel  = mode != null ? (MODE_LABEL[mode] ?? `Mode ${mode}`) : '—';
  const modeColor  = mode != null ? (MODE_COLOR[mode]  ?? 'bg-gray-700 text-gray-200') : 'bg-gray-800 text-gray-500';

  const tempColor  = thermalTemp > 70 ? 'text-rose-400' : thermalTemp > 60 ? 'text-amber-400' : 'text-emerald-400';
  const cpuColor   = cpuLoad     > 75 ? 'text-rose-400' : cpuLoad     > 50 ? 'text-amber-400' : 'text-emerald-400';

  return (
    <div className="bg-gray-900 border border-white/10 rounded-lg overflow-hidden">

      {/* Header */}
      <div className="flex items-center justify-between px-4 py-2.5 bg-gray-800 border-b border-white/10">
        <div className="flex items-center gap-2">
          <Activity size={14} className="text-sky-400" />
          <span className="text-sm font-bold tracking-wide text-white uppercase">
            System Status
          </span>
          <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 text-white/40 font-medium">
            READ-ONLY
          </span>
        </div>
        {offline && (
          <span className="text-[11px] text-rose-400 font-semibold">Offline</span>
        )}
      </div>

      <div className="p-4 space-y-4">

        {/* Active mode badge */}
        <div className="space-y-1.5">
          <div className="flex items-center gap-2 flex-wrap">
            <span className={`inline-flex items-center gap-1.5 px-3 py-1 rounded-md text-xs font-bold ${modeColor}`}>
              {mode != null && (
                <span className="opacity-60 text-[10px] font-mono">#{mode}</span>
              )}
              {modeLabel}
            </span>

            {manualOverride && (
              <span className="inline-flex items-center gap-1 px-2 py-0.5 rounded bg-amber-600/30 border border-amber-500/40 text-amber-300 text-[10px] font-semibold">
                <Shield size={10} />
                Manual Override
              </span>
            )}
          </div>

          {description && (
            <p className="text-xs text-white/50 leading-snug pl-0.5">{description}</p>
          )}
        </div>

        {/* Telemetry row */}
        <div className="flex flex-wrap gap-x-5 gap-y-2 pt-3 border-t border-white/8">
          <Stat
            icon={<Thermometer size={13} />}
            label="Temp"
            value={thermalTemp > 0 ? `${thermalTemp.toFixed(1)}°C` : '—'}
            colorClass={tempColor}
          />
          <Stat
            icon={<Cpu size={13} />}
            label="CPU"
            value={cpuLoad > 0 ? `${cpuLoad.toFixed(1)}%` : '—'}
            colorClass={cpuColor}
          />
          <Stat
            icon={<Radio size={13} />}
            label="Orin"
            value={orinAvailable ? 'Available' : 'Unavailable'}
            colorClass={orinAvailable ? 'text-emerald-400' : 'text-white/30'}
          />
          {hybridMode && (
            <Stat
              icon={<Activity size={13} />}
              label="System"
              value={hybridMode.replace(/_/g, ' ')}
              colorClass="text-white/60"
            />
          )}
        </div>

        {/* Throttling warning — only shown when active */}
        {throttling && (
          <div className="flex items-center gap-2 px-3 py-2 rounded-md bg-amber-600/20 border border-amber-500/40">
            <AlertTriangle size={13} className="text-amber-400 shrink-0" />
            <span className="text-xs text-amber-300 font-semibold">
              Thermal / CPU throttle active — CV pipeline may be rate-limited
            </span>
          </div>
        )}

      </div>
    </div>
  );
}
