'use client';

import React, { useState, useCallback } from 'react';
import { useSystemMode } from '@/hooks/useSystemMode';
import {
  Video, Activity, Crosshair, User, QrCode,
  Circle, Zap, Map, Shield, Compass, Camera, Eye,
  ChevronDown, ChevronUp, ScanEye, AlertTriangle,
} from 'lucide-react';

/* ------------------------------------------------------------------ */
/* Mode definitions                                                    */
/* ------------------------------------------------------------------ */

interface ModeSpec {
  id: number;
  name: string;
  short: string;
  icon: React.ElementType;
  cpu: 0 | 1 | 2 | 3;
  piSafe: boolean;
  orinOnly?: boolean;
  description: string;
  /** tailwind color class for the icon accent */
  accent: string;
  /** tailwind ring/border color for active state */
  activeRing: string;
  /** tailwind bg for active state */
  activeBg: string;
}

const MODES: ModeSpec[] = [
  {
    id: 0,
    name: 'Raw Stream',
    short: 'Raw',
    icon: Video,
    cpu: 0,
    piSafe: true,
    description: 'Direct MJPEG feed — no vision processing overhead',
    accent: 'text-slate-300',
    activeRing: 'ring-slate-400 border-slate-400',
    activeBg: 'bg-slate-800/60',
  },
  {
    id: 1,
    name: 'Motion Detection',
    short: 'Motion',
    icon: Activity,
    cpu: 1,
    piSafe: true,
    description: 'Frame differencing flags movement in the scene',
    accent: 'text-sky-400',
    activeRing: 'ring-sky-500 border-sky-500',
    activeBg: 'bg-sky-900/40',
  },
  {
    id: 2,
    name: 'Object Tracking',
    short: 'Tracking',
    icon: Crosshair,
    cpu: 2,
    piSafe: true,
    description: 'KCF tracker locks onto and follows a selected target',
    accent: 'text-teal-400',
    activeRing: 'ring-teal-500 border-teal-500',
    activeBg: 'bg-teal-900/40',
  },
  {
    id: 3,
    name: 'Face Detection',
    short: 'Faces',
    icon: User,
    cpu: 2,
    piSafe: true,
    description: 'Haar cascade detects & recognises faces in frame',
    accent: 'text-violet-400',
    activeRing: 'ring-violet-500 border-violet-500',
    activeBg: 'bg-violet-900/40',
  },
  {
    id: 4,
    name: 'ArUco Markers',
    short: 'ArUco',
    icon: QrCode,
    cpu: 1,
    piSafe: true,
    description: 'Reads ArUco fiducial markers for navigation & docking',
    accent: 'text-amber-400',
    activeRing: 'ring-amber-500 border-amber-500',
    activeBg: 'bg-amber-900/40',
  },
  {
    id: 5,
    name: 'Pi Record',
    short: 'Pi Rec',
    icon: Circle,
    cpu: 1,
    piSafe: true,
    description: 'Server-side recording — saves video to the Pi\'s disk (not your browser)',
    accent: 'text-rose-400',
    activeRing: 'ring-rose-500 border-rose-500',
    activeBg: 'bg-rose-900/40',
  },
  {
    id: 6,
    name: 'YOLOv8',
    short: 'YOLO',
    icon: Zap,
    cpu: 3,
    piSafe: false,
    orinOnly: true,
    description: 'GPU-accelerated object detection via YOLOv8 — requires Jetson Orin',
    accent: 'text-indigo-400',
    activeRing: 'ring-indigo-500 border-indigo-500',
    activeBg: 'bg-indigo-900/40',
  },
  {
    id: 7,
    name: 'Full Autonomy',
    short: 'Autonomy',
    icon: Map,
    cpu: 3,
    piSafe: false,
    orinOnly: true,
    description: 'Full navigation AI brain with SLAM — requires Jetson Orin',
    accent: 'text-fuchsia-400',
    activeRing: 'ring-fuchsia-500 border-fuchsia-500',
    activeBg: 'bg-fuchsia-900/40',
  },
  {
    id: 8,
    name: 'Obstacle Detect',
    short: 'Obstacle',
    icon: AlertTriangle,
    cpu: 0,
    piSafe: true,
    description: 'Ultrasonic proximity box — color-coded distance warning with danger flash',
    accent: 'text-orange-400',
    activeRing: 'ring-orange-500 border-orange-500',
    activeBg: 'bg-orange-900/40',
  },
];

/* ------------------------------------------------------------------ */
/* Combo presets                                                       */
/* ------------------------------------------------------------------ */

interface ComboSpec {
  id: string;
  name: string;
  icon: React.ElementType;
  involves: number[];
  mapTo: number;
  description: string;
  accent: string;
}

const COMBOS: ComboSpec[] = [
  {
    id: 'sentry',
    name: 'Sentry',
    icon: Shield,
    involves: [1, 3],
    mapTo: 1,
    description: 'Motion triggers face sweep — minimal combined CPU',
    accent: 'text-sky-400',
  },
  {
    id: 'nav-assist',
    name: 'Nav Assist',
    icon: Compass,
    involves: [4, 1],
    mapTo: 4,
    description: 'ArUco waypoints + motion awareness',
    accent: 'text-amber-400',
  },
  {
    id: 'security',
    name: 'Security Cam',
    icon: Camera,
    involves: [1, 5],
    mapTo: 1,
    description: 'Auto-records when motion is detected',
    accent: 'text-rose-400',
  },
  {
    id: 'patrol',
    name: 'Patrol',
    icon: Eye,
    involves: [1, 4, 5],
    mapTo: 1,
    description: 'Motion + ArUco + recording — full lightweight patrol',
    accent: 'text-teal-400',
  },
];

/* ------------------------------------------------------------------ */
/* CPU indicator                                                       */
/* ------------------------------------------------------------------ */

const CPU_META = [
  { label: 'Minimal', dotColor: 'bg-emerald-400', textColor: 'text-emerald-400' },
  { label: 'Low',     dotColor: 'bg-green-400',   textColor: 'text-green-400'   },
  { label: 'Medium',  dotColor: 'bg-amber-400',   textColor: 'text-amber-400'   },
  { label: 'High',    dotColor: 'bg-rose-400',    textColor: 'text-rose-400'    },
] as const;

function CpuBar({ level }: { level: 0 | 1 | 2 | 3 }) {
  const { dotColor, label, textColor } = CPU_META[level];
  return (
    <span className={`flex items-center gap-1 text-[11px] font-semibold ${textColor}`}>
      <span className="flex gap-0.5">
        {[0, 1, 2].map(i => (
          <span
            key={i}
            className={`w-2 h-2 rounded-sm ${i < (level === 0 ? 1 : level) ? dotColor : 'bg-white/10'}`}
          />
        ))}
      </span>
      {label}
    </span>
  );
}

/* ------------------------------------------------------------------ */
/* Main component                                                      */
/* ------------------------------------------------------------------ */

export default function VisionModePanel() {
  // Single shared polling source — no independent poll loop here.
  const { mode: currentMode, orinAvailable, description: backendDescription, refresh } = useSystemMode();

  const [setting, setSetting] = useState(false);
  const [showCombos, setShowCombos] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const setMode = useCallback(async (modeId: number) => {
    if (setting) return;
    setSetting(true);
    setError(null);
    try {
      const res = await fetch('/api/system/mode/set', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: modeId }),
      });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      // Trigger immediate re-fetch so the UI reflects the new mode without
      // waiting for the next 15 s poll cycle.
      refresh();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to set mode');
    } finally {
      setSetting(false);
    }
  }, [setting, refresh]);

  const activeMode = MODES.find(m => m.id === currentMode);

  return (
    <div className="mt-1 bg-gray-900 border border-white/10 rounded-lg overflow-hidden">

      {/* ── Header ────────────────────────────────────────────────── */}
      <div className="flex items-center justify-between px-4 py-2.5 bg-gray-800 border-b border-white/10">
        <div className="flex items-center gap-2">
          <ScanEye size={15} className="text-violet-400" />
          <span className="text-sm font-bold tracking-wide text-white uppercase">
            Vision Mode
          </span>
        </div>

        {activeMode ? (
          <div className="flex items-center gap-2">
            {/* Active mode badge */}
            <span className={`flex items-center gap-1.5 px-2.5 py-1 rounded-md text-xs font-semibold ${activeMode.activeBg} border ${activeMode.activeRing.split(' ')[1]} text-white`}>
              <activeMode.icon size={12} className={activeMode.accent} />
              {activeMode.name}
            </span>
            <CpuBar level={activeMode.cpu} />
          </div>
        ) : (
          <span className="text-xs text-white/40 italic">Offline</span>
        )}
      </div>

      {/* ── Mode grid ─────────────────────────────────────────────── */}
      <div className="p-3">
        <div className="grid grid-cols-4 sm:grid-cols-8 gap-2">
          {MODES.map(mode => {
            const Icon = mode.icon;
            const isActive = currentMode === mode.id;
            const disabled = (mode.orinOnly && !orinAvailable) || setting;

            return (
              <button
                key={mode.id}
                onClick={() => !disabled && setMode(mode.id)}
                disabled={disabled}
                title={mode.description}
                className={[
                  'relative flex flex-col items-center justify-center gap-1.5 rounded-lg py-3 px-2 border transition-all duration-150',
                  'focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-violet-500',
                  isActive
                    ? `${mode.activeBg} border ring-1 ${mode.activeRing} shadow-lg`
                    : disabled
                      ? 'bg-gray-800/40 border-white/5 opacity-35 cursor-not-allowed'
                      : 'bg-gray-800 border-white/10 hover:bg-gray-700 hover:border-white/20 cursor-pointer',
                ].join(' ')}
              >
                <Icon
                  size={18}
                  className={isActive ? mode.accent : disabled ? 'text-white/30' : 'text-white/60'}
                />
                <span className={`text-[11px] font-semibold leading-tight text-center ${
                  isActive ? 'text-white' : disabled ? 'text-white/30' : 'text-white/70'
                }`}>
                  {mode.short}
                </span>

                {/* Orin badge */}
                {mode.orinOnly && (
                  <span className="absolute -top-1.5 -right-1.5 text-[8px] font-bold bg-indigo-600 text-white rounded px-1 py-0.5 leading-none shadow">
                    ORIN
                  </span>
                )}

                {/* Active indicator dot */}
                {isActive && (
                  <span className="absolute bottom-1 left-1/2 -translate-x-1/2 w-1.5 h-1.5 rounded-full bg-white shadow-[0_0_4px_rgba(255,255,255,0.8)]" />
                )}
              </button>
            );
          })}
        </div>

        {/* Active mode description — sourced from backend (blueprint §10) */}
        <div className="mt-2.5 min-h-[18px]">
          {activeMode && (
            <p className="text-xs text-white/60 leading-snug">
              <span className={`font-semibold ${activeMode.accent}`}>{activeMode.name}:</span>{' '}
              {backendDescription || activeMode.description}
            </p>
          )}
        </div>

        {/* Error */}
        {error && (
          <p className="mt-1.5 text-xs text-rose-400 font-medium">{error}</p>
        )}
      </div>

      {/* ── Combos section ────────────────────────────────────────── */}
      <div className="border-t border-white/8">
        <button
          onClick={() => setShowCombos(s => !s)}
          className="w-full flex items-center justify-between px-4 py-2.5 bg-gray-800/50 hover:bg-gray-800 transition-colors text-left"
        >
          <div className="flex items-center gap-2">
            <span className="text-xs font-bold uppercase tracking-wider text-white/70">
              Presets & Combos
            </span>
            <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 text-white/50 font-medium">
              {COMBOS.length}
            </span>
          </div>
          {showCombos
            ? <ChevronUp size={14} className="text-white/50" />
            : <ChevronDown size={14} className="text-white/50" />
          }
        </button>

        {showCombos && (
          <div className="p-3 grid grid-cols-2 sm:grid-cols-4 gap-2">
            {COMBOS.map(combo => {
              const Icon = combo.icon;
              const isActive = currentMode === combo.mapTo;
              return (
                <button
                  key={combo.id}
                  onClick={() => !setting && setMode(combo.mapTo)}
                  disabled={setting}
                  title={combo.description}
                  className={[
                    'flex flex-col gap-1.5 rounded-lg p-3 text-left border transition-all',
                    isActive
                      ? 'bg-violet-900/40 border-violet-500/60 ring-1 ring-violet-500'
                      : 'bg-gray-800 border-white/10 hover:bg-gray-700 hover:border-white/20',
                  ].join(' ')}
                >
                  <div className="flex items-center gap-2">
                    <Icon size={14} className={isActive ? 'text-violet-300' : combo.accent} />
                    <span className={`text-xs font-bold ${isActive ? 'text-white' : 'text-white/90'}`}>
                      {combo.name}
                    </span>
                  </div>
                  <p className="text-[11px] text-white/55 leading-snug">
                    {combo.description}
                  </p>
                  <div className="flex gap-1 flex-wrap mt-0.5">
                    {combo.involves.map(id => {
                      const m = MODES.find(mm => mm.id === id);
                      return m ? (
                        <span key={id} className="text-[9px] font-semibold px-1 py-0.5 rounded bg-white/10 text-white/50">
                          {m.short}
                        </span>
                      ) : null;
                    })}
                    <span className="text-[9px] text-white/25 ml-auto self-center">multi-mode soon</span>
                  </div>
                </button>
              );
            })}
          </div>
        )}
      </div>

    </div>
  );
}
