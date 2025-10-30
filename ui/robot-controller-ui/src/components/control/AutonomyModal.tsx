/*
# File: /src/components/control/AutonomyModal.tsx
# Summary:
#   Simple-first autonomy modal with debounced live updates:
#     • Top: Mode + Start/Stop, Speed, 2 safety toggles, Waypoints
#     • Bottom: one "Advanced" fold-out (Navigation, Vision, Behavior, Mapping, Safety, Presets)
#   - Backwards compatible with onStart/onStop/onDock/onSetWaypoint
#   - Optional onUpdate(params) is debounced (250ms) to avoid backend spam
#   - Duplicate param payloads are suppressed
#   - Optional liveApply (default: true). "Apply Params" flushes pending updates.
#   - High-contrast colors (dark bg, bright text/borders)
*/

'use client';

import React, { useMemo, useRef, useState } from 'react';
import { Bot, Play, Square, Gauge, Shield, Zap, Flag, Crosshair, Settings2, Save, Upload, Info, HelpCircle, Eye, User, QrCode } from 'lucide-react';

import { Button } from '@/components/ui/button';
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from '@/components/ui/dialog';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Switch } from '@/components/ui/switch';
import { Slider } from '@/components/ui/slider';
import { Input } from '@/components/ui/input';
import { Badge } from '@/components/ui/badge';
import { Card, CardContent } from '@/components/ui/card';
import { useDebouncedCallback } from '@/utils/debounce';

/* ------------------------------ Types ------------------------------ */

export type AutonomyMode =
  | 'idle' | 'patrol' | 'follow' | 'dock' | 'line_track' // legacy
  | 'line_follow' | 'avoid_obstacles' | 'edge_detect' | 'waypoints'
  | 'color_track' | 'aruco' | 'person_follow' | 'scan_servo';

export type AutonomyParams = {
  // legacy
  speedPct: number;        // 0–100
  aggressiveness: number;  // 0–100
  obstacleAvoidance: boolean;
  laneKeeping: boolean;
  headlights: boolean;     // interpreted as "Auto-lights" permission

  // nav
  avoidStopDistM: number;
  line_kP: number;
  searchYawRate: number;

  // vision (HSV)
  hsvLow: [number, number, number];
  hsvHigh:[number, number, number];
  minArea: number;
  maxArea: number;

  // behavior priorities
  priorities: Array<'safety'|'avoid'|'edge'|'line'|'waypoints'|'vision'>;

  // mapping
  gridEnabled: boolean;
  gridCellSizeM: number;
  gridDecay: number;

  // safety
  batteryMinPct: number;

  // computer vision
  cvEnabled: boolean;
  cvMode: 'color_track' | 'face_follow' | 'person_follow' | 'aruco_track' | 'motion_detect' | 'object_detect';
  cvConfidenceThreshold: number; // 0-100
  cvMaxDetections: number;
  cvShowBoundingBoxes: boolean;
  cvTrackObjects: boolean;
  cvTrackerType: 'CSRT' | 'KCF' | 'MIL';
};

export type AutonomyModalProps = {
  className?: string;
  initialMode?: AutonomyMode;
  initialParams?: Partial<AutonomyParams>;
  onStart?: (mode: AutonomyMode, params: AutonomyParams) => Promise<void> | void;
  onStop?: () => Promise<void> | void;
  onDock?: () => Promise<void> | void;
  onSetWaypoint?: (label: string, lat: number, lon: number) => Promise<void> | void;
  onUpdate?: (params: AutonomyParams) => Promise<void> | void; // optional live updates
  connected?: boolean;
  autonomyActive?: boolean;
  batteryPct?: number;
  triggerLabel?: string;
  /** If true (default), param changes auto-push via debounce. If false, only "Apply Params" sends. */
  liveApply?: boolean;
};

/* -------------------------------- Defaults -------------------------------- */

const defaults: AutonomyParams = {
  // legacy/simple
  speedPct: 40,
  aggressiveness: 30,
  obstacleAvoidance: true,
  laneKeeping: true,
  headlights: false,

  // nav
  avoidStopDistM: 0.25,
  line_kP: 0.9,
  searchYawRate: 35,

  // vision
  hsvLow: [0, 120, 120],
  hsvHigh:[10, 255, 255],
  minArea: 500,
  maxArea: 25000,

  // behavior order (top = highest priority)
  priorities: ['safety','avoid','edge','line','waypoints','vision'],

  // mapping
  gridEnabled: false,
  gridCellSizeM: 0.15,
  gridDecay: 0.04,

  // safety
  batteryMinPct: 15,

  // computer vision
  cvEnabled: false,
  cvMode: 'color_track',
  cvConfidenceThreshold: 70,
  cvMaxDetections: 5,
  cvShowBoundingBoxes: true,
  cvTrackObjects: true,
  cvTrackerType: 'CSRT',
};

/* -------------------------------- Component -------------------------------- */

export default function AutonomyModal({
  className,
  initialMode = 'idle',
  initialParams = {},
  onStart,
  onStop,
  onDock,
  onSetWaypoint,
  onUpdate,
  connected = true,
  autonomyActive = false,
  batteryPct = 100,
  triggerLabel = 'Autonomy',
  liveApply = true,
}: AutonomyModalProps) {
  const [open, setOpen] = useState(false);
  const [mode, setMode] = useState<AutonomyMode>(initialMode);
  const [params, setParams] = useState<AutonomyParams>({ ...defaults, ...initialParams });

  // Waypoints (client-side preview + push to backend via onSetWaypoint)
  const [wpLabel, setWpLabel] = useState('Home');
  const [wpLat, setWpLat] = useState<string>('');
  const [wpLon, setWpLon] = useState<string>('');
  const [waypoints, setWaypoints] = useState<Array<{label:string, lat:number, lon:number}>>([]);

  // UI
  const [busy, setBusy] = useState(false);
  const [showAdvanced, setShowAdvanced] = useState(false);
  const fileRef = useRef<HTMLInputElement>(null);

  const statusTone = useMemo(() => {
    if (!connected) return 'bg-red-500/25 text-red-200 border-red-500/60';
    if (autonomyActive) return 'bg-emerald-600/25 text-emerald-200 border-emerald-500/70';
    return 'bg-amber-500/25 text-amber-200 border-amber-500/70';
  }, [connected, autonomyActive]);

  /* --------------------------- Debounced onUpdate --------------------------- */
  // Suppress duplicate payloads (same JSON)
  const lastSentRef = useRef<string>('');

  const debouncedPush = useDebouncedCallback(
    async (next: AutonomyParams) => {
      if (!onUpdate) return;
      const json = safeStableStringify(next);
      if (json === lastSentRef.current) return;
      lastSentRef.current = json;
      try { await onUpdate(next); }
      catch (e) { console.warn('autonomy onUpdate failed:', e); }
    },
    250, // ms
    { trailing: true, leading: false },
    [onUpdate]
  );

  // Helper: update local state and schedule debounced backend push (if liveApply)
  function setParam<K extends keyof AutonomyParams>(key: K, value: AutonomyParams[K]) {
    setParams(prev => {
      const next = { ...prev, [key]: value };
      if (liveApply && onUpdate) debouncedPush(next);
      return next;
    });
  }

  /* -------------------------------- Handlers -------------------------------- */

  async function handleStart() {
    setBusy(true);
    try { await onStart?.(canonicalizeMode(mode), params); }
    catch (e) { console.warn('autonomy start failed:', e); }
    finally { setBusy(false); }
  }

  async function handleStop()  {
    setBusy(true);
    try { await onStop?.(); }
    catch (e) { console.warn('autonomy stop failed:', e); }
    finally { setBusy(false); }
  }

  async function handleDock()  {
    setBusy(true);
    try { setMode('dock'); await onDock?.(); }
    catch (e) { console.warn('autonomy dock failed:', e); }
    finally { setBusy(false); }
  }

  async function handleSetWaypoint() {
    const lat = Number(wpLat), lon = Number(wpLon);
    const valid =
      wpLabel.trim().length > 0 &&
      Number.isFinite(lat) && Number.isFinite(lon) &&
      lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;
    if (!valid) return;

    setBusy(true);
    try {
      await onSetWaypoint?.(wpLabel.trim(), lat, lon);
      setWaypoints((prev) => [...prev, { label: wpLabel.trim(), lat, lon }]);
      setWpLabel(''); setWpLat(''); setWpLon('');
    } catch (e) {
      console.warn('set waypoint failed:', e);
    } finally {
      setBusy(false);
    }
  }

  async function handleApplyParams() {
    if (!onUpdate) return;
    setBusy(true);
    try {
      if (liveApply) {
        // Force any pending debounced call to run immediately
        debouncedPush.flush();
      } else {
        lastSentRef.current = ''; // ensure not suppressed
        await onUpdate(params);
      }
    } catch (e) {
      console.warn('apply params failed:', e);
    } finally {
      setBusy(false);
    }
  }

  // presets
  function downloadPresets() {
    try {
      const blob = new Blob([JSON.stringify({ mode, params, waypoints }, null, 2)], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a'); a.href = url; a.download = 'autonomy-preset.json'; a.click();
      URL.revokeObjectURL(url);
    } catch (e) {
      console.warn('export presets failed:', e);
    }
  }
  function openFileDialog() { fileRef.current?.click(); }
  function onFileSelected(e: React.ChangeEvent<HTMLInputElement>) {
    const f = e.target.files?.[0]; if (!f) return;
    const reader = new FileReader();
    reader.onload = () => {
      try {
        const json = JSON.parse(String(reader.result || '{}'));
        if (json.mode) setMode(json.mode);
        if (json.params) {
          setParams((p) => {
            const next = { ...p, ...json.params };
            if (liveApply && onUpdate) debouncedPush(next);
            return next;
          });
        }
        if (Array.isArray(json.waypoints)) setWaypoints(json.waypoints);
      } catch (err) {
        console.warn('import presets failed:', err);
      }
    };
    reader.readAsText(f);
    e.target.value = '';
  }

  /* --------------------------------- Render --------------------------------- */

  const waypointError =
    (wpLat && !isFinite(+wpLat)) || (wpLon && !isFinite(+wpLon)) ||
    (+wpLat < -90 || +wpLat > 90 || +wpLon < -180 || +wpLon > 180);

  return (
    <div className={className}>
      <Dialog open={open} onOpenChange={setOpen}>
        <DialogTrigger asChild>
          <Button
            className="
              mt-3 gap-2 border-0
              bg-gradient-to-r from-emerald-600 via-teal-600 to-cyan-600
              hover:from-emerald-500 hover:via-teal-500 hover:to-cyan-500
              text-white shadow-lg shadow-emerald-500/25
              focus-visible:ring-2 focus-visible:ring-emerald-400/70
            "
            aria-label="Open Autonomy settings"
          >
            <Bot className="h-4 w-4" /> {triggerLabel}
          </Button>
        </DialogTrigger>

        <DialogContent className="sm:max-w-xl max-h-[85vh] p-0 overflow-hidden border border-neutral-800 bg-neutral-950 text-neutral-100 flex flex-col">
          <DialogHeader className="px-5 pt-5 pb-2 flex-shrink-0">
            <DialogTitle className="flex items-center gap-2 text-lg">
              <Settings2 className="h-5 w-5" /> Autonomy Control
            </DialogTitle>
            <p className="text-xs text-neutral-400 mt-1">
              Choose a mode, adjust settings, and start your robot's autonomous behavior
            </p>
          </DialogHeader>

          {/* Status */}
          <div className="px-5 pb-3 flex items-center justify-between gap-2 flex-shrink-0">
            <Badge variant="outline" className={`${statusTone} border`} aria-live="polite">
              {connected ? (autonomyActive ? 'Active' : 'Connected') : 'Disconnected'}
            </Badge>
            <div className="flex items-center gap-2 text-xs text-neutral-100">
              <Gauge className="h-4 w-4" aria-hidden /> {batteryPct}%
            </div>
          </div>

          <div className="px-5 pb-5 grid grid-cols-1 gap-3 overflow-y-auto flex-1 min-h-0">
            {/* BASIC: Mode + Start/Stop */}
            <Card className="bg-neutral-900/80 border-neutral-800">
              <CardContent className="p-3 grid gap-3">
                <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 items-end">
                  <div>
                    <div className="flex items-center gap-1 mb-1">
                      <label className="text-xs text-neutral-200 font-medium">Autonomy Mode</label>
                      <div className="group relative">
                        <HelpCircle className="h-3 w-3 text-neutral-400 cursor-help" />
                        <div className="absolute left-0 top-4 w-48 bg-neutral-900 border border-neutral-700 rounded-md p-2 text-xs text-neutral-200 opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all z-50 shadow-lg">
                          <strong>Mode Options:</strong><br/>
                          • <strong>Line Follow:</strong> Follows marked lines on the ground<br/>
                          • <strong>Avoid Obstacles:</strong> Drives while avoiding objects<br/>
                          • <strong>Patrol:</strong> Follows a preset route<br/>
                          • <strong>Waypoints:</strong> Navigates to specific locations<br/>
                          • <strong>Color Track:</strong> Tracks objects by color
                        </div>
                      </div>
                    </div>
                    <Select value={mode} onValueChange={(v: string) => setMode(v as AutonomyMode)}>
                      <SelectTrigger className="mt-1 bg-neutral-950 border-neutral-800 text-neutral-100">
                        <SelectValue placeholder="Select mode" />
                      </SelectTrigger>
                      <SelectContent className="bg-neutral-950 border-neutral-800 text-neutral-100">
                        <SelectItem value="idle">
                          <div>
                            <div className="font-medium">Idle</div>
                            <div className="text-xs text-neutral-400">Robot stays still</div>
                          </div>
                        </SelectItem>
                        <SelectItem value="line_follow">
                          <div>
                            <div className="font-medium">Line Follow</div>
                            <div className="text-xs text-neutral-400">Follows marked lines</div>
                          </div>
                        </SelectItem>
                        <SelectItem value="avoid_obstacles">
                          <div>
                            <div className="font-medium">Avoid Obstacles</div>
                            <div className="text-xs text-neutral-400">Drives safely around objects</div>
                          </div>
                        </SelectItem>
                        <SelectItem value="patrol">
                          <div>
                            <div className="font-medium">Patrol</div>
                            <div className="text-xs text-neutral-400">Follows a preset route</div>
                          </div>
                        </SelectItem>
                        <SelectItem value="waypoints">
                          <div>
                            <div className="font-medium">Waypoints</div>
                            <div className="text-xs text-neutral-400">Navigate to specific locations</div>
                          </div>
                        </SelectItem>
                        <SelectItem value="line_track">Line Track (Legacy)</SelectItem>
                        <SelectItem value="follow">Follow</SelectItem>
                        <SelectItem value="color_track">Color Track</SelectItem>
                        <SelectItem value="dock">Dock</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                  <div className="flex gap-2">
                    <Button disabled={busy || !connected} onClick={handleStart} className="gap-2 flex-1" aria-busy={busy}>
                      <Play className="h-4 w-4" /> Start
                    </Button>
                    <Button disabled={busy} variant="destructive" onClick={handleStop} className="gap-2 flex-1" aria-busy={busy}>
                      <Square className="h-4 w-4" /> Stop
                    </Button>
                  </div>
                </div>

                {/* BASIC: Speed + two obvious toggles */}
                <div className="space-y-3 pt-2 border-t border-neutral-800/50">
                  <div>
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-1">
                        <span className="text-xs font-medium text-neutral-200">Speed</span>
                        <div className="group relative">
                          <HelpCircle className="h-3 w-3 text-neutral-400 cursor-help" />
                          <div className="absolute left-0 top-4 w-56 bg-neutral-900 border border-neutral-700 rounded-md p-2 text-xs text-neutral-200 opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all z-50 shadow-lg">
                            Controls how fast the robot moves (0-100%). Start with 30-40% for safe testing.
                          </div>
                        </div>
                      </div>
                      <span className="text-xs font-bold text-neutral-100">{params.speedPct}%</span>
                    </div>
                    <Slider
                      value={[params.speedPct]}
                      onValueChange={(v: number[]) => setParam('speedPct', v[0] ?? 0)}
                      min={0}
                      max={100}
                      step={5}
                      className="mt-2"
                      aria-label="Speed"
                    />
                    <div className="flex justify-between text-[10px] text-neutral-500 mt-1">
                      <span>Slow</span>
                      <span>Fast</span>
                    </div>
                </div>

                  <div>
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-1">
                        <span className="text-xs font-medium text-neutral-200">Response Speed</span>
                        <div className="group relative">
                          <HelpCircle className="h-3 w-3 text-neutral-400 cursor-help" />
                          <div className="absolute left-0 top-4 w-56 bg-neutral-900 border border-neutral-700 rounded-md p-2 text-xs text-neutral-200 opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all z-50 shadow-lg">
                            How quickly the robot reacts to changes. Lower = smoother, Higher = more responsive.
                          </div>
                        </div>
                      </div>
                      <span className="text-xs font-bold text-neutral-100">{params.aggressiveness}%</span>
                    </div>
                    <Slider
                      value={[params.aggressiveness]}
                      onValueChange={(v: number[]) => setParam('aggressiveness', v[0] ?? 0)}
                      min={0}
                      max={100}
                      step={5}
                      className="mt-2"
                      aria-label="Response Speed"
                    />
                    <div className="flex justify-between text-[10px] text-neutral-500 mt-1">
                      <span>Smooth</span>
                      <span>Responsive</span>
                    </div>
                  </div>
                </div>

                <div className="space-y-2 pt-2 border-t border-neutral-800/50">
                  <div className="text-xs font-medium text-neutral-300 mb-2">Safety Features</div>
                  <ToggleRow
                    icon={<Shield className="h-4 w-4 text-emerald-400" />}
                    label="Obstacle Avoidance"
                    description="Robot will stop or turn when it detects objects in its path"
                    checked={params.obstacleAvoidance}
                    onCheckedChange={(v) => setParam('obstacleAvoidance', v)}
                  />
                  <ToggleRow
                    icon={<Zap className="h-4 w-4 text-yellow-400" />}
                    label="Auto-lights"
                    description="Automatically adjust LED lighting for better visibility"
                    checked={params.headlights}
                    onCheckedChange={(v) => setParam('headlights', v)}
                  />
                </div>
              </CardContent>
            </Card>

            {/* BASIC: Waypoints - Only show if waypoints mode is selected */}
            {mode === 'waypoints' && (
              <Card className="bg-neutral-900/80 border-neutral-800">
                <CardContent className="p-3 grid gap-3">
                <div className="flex items-center gap-2 text-sm font-medium text-neutral-100">
                    <Flag className="h-4 w-4" /> Navigation Waypoints
                </div>
                  <p className="text-xs text-neutral-400">
                    Set specific locations for your robot to navigate to. GPS coordinates required.
                  </p>
                <div className="grid grid-cols-1 sm:grid-cols-4 gap-3">
                    <Input value={wpLabel} onChange={(e) => setWpLabel(e.target.value)} placeholder="Location name"
                         className="bg-neutral-950 border-neutral-800 placeholder:text-neutral-500" />
                    <Input value={wpLat}   onChange={(e) => setWpLat(e.target.value)}   placeholder="Latitude"
                         inputMode="decimal" className="bg-neutral-950 border-neutral-800 placeholder:text-neutral-500" />
                    <Input value={wpLon}   onChange={(e) => setWpLon(e.target.value)}   placeholder="Longitude"
                         inputMode="decimal" className="bg-neutral-950 border-neutral-800 placeholder:text-neutral-500" />
                  <Button
                    disabled={busy || waypointError || !wpLat || !wpLon || !wpLabel.trim()}
                    onClick={handleSetWaypoint}
                    className="gap-2"
                    aria-disabled={busy || waypointError}
                      title={waypointError ? 'Latitude must be -90 to 90, Longitude -180 to 180' : 'Add waypoint'}
                  >
                    <Crosshair className="h-4 w-4" /> Add
                  </Button>
                </div>
                {waypointError && (
                    <div className="text-[11px] text-red-300 flex items-center gap-1">
                      <Info className="h-3 w-3" /> Invalid coordinates. Latitude: -90 to 90, Longitude: -180 to 180.
                    </div>
                )}

                {waypoints.length > 0 && (
                    <div className="mt-2">
                      <div className="text-xs text-neutral-400 mb-2">Saved Waypoints ({waypoints.length}):</div>
                      <div className="text-xs text-neutral-300 grid gap-1">
                    {waypoints.map((w, i) => (
                      <div key={`${w.label}-${i}`} className="flex items-center justify-between rounded border border-neutral-800 bg-neutral-950 px-2 py-1">
                        <span className="font-medium text-neutral-100">{w.label}</span>
                            <span className="tabular-nums text-neutral-300 text-[10px]">{w.lat.toFixed(5)}, {w.lon.toFixed(5)}</span>
                      </div>
                    ))}
                      </div>
                  </div>
                  )}
                </CardContent>
              </Card>
            )}

            {/* Computer Vision Section */}
            <Card className="bg-neutral-900/80 border-neutral-800">
              <CardContent className="p-3 grid gap-3">
                <div className="flex items-center gap-2 text-sm font-medium text-neutral-100">
                  <Eye className="h-4 w-4 text-cyan-400" /> Computer Vision
                </div>
                <p className="text-xs text-neutral-400">
                  Enable camera-based detection and tracking. The robot uses its camera to find and follow objects, faces, or markers.
                </p>

                <ToggleRow
                  icon={<Eye className="h-4 w-4 text-cyan-400" />}
                  label="Enable Computer Vision"
                  description="Turn on camera-based detection for vision-powered autonomy"
                  checked={params.cvEnabled}
                  onCheckedChange={(v) => setParam('cvEnabled', v)}
                />

                {params.cvEnabled && (
                  <>
                    <div>
                      <div className="flex items-center gap-1 mb-1">
                        <label className="text-xs text-neutral-200 font-medium">Vision Mode</label>
                        <div className="group relative">
                          <HelpCircle className="h-3 w-3 text-neutral-400 cursor-help" />
                          <div className="absolute left-0 top-4 w-64 bg-neutral-900 border border-neutral-700 rounded-md p-2 text-xs text-neutral-200 opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all z-50 shadow-lg">
                            <strong>Vision Modes:</strong><br/>
                            • <strong>Color Track:</strong> Follow objects by color<br/>
                            • <strong>Face Follow:</strong> Track and follow faces<br/>
                            • <strong>Person Follow:</strong> Detect and follow people<br/>
                            • <strong>ArUco Track:</strong> Follow ArUco marker tags<br/>
                            • <strong>Motion Detect:</strong> React to movement<br/>
                            • <strong>Object Detect:</strong> Detect any objects
                          </div>
                        </div>
                      </div>
                      <Select 
                        value={params.cvMode} 
                        onValueChange={(v) => setParam('cvMode', v as AutonomyParams['cvMode'])}
                      >
                        <SelectTrigger className="mt-1 bg-neutral-950 border-neutral-800 text-neutral-100">
                          <SelectValue />
                        </SelectTrigger>
                        <SelectContent className="bg-neutral-950 border-neutral-800 text-neutral-100">
                          <SelectItem value="color_track">
                            <div>
                              <div className="font-medium">Color Track</div>
                              <div className="text-xs text-neutral-400">Follow colored objects</div>
                            </div>
                          </SelectItem>
                          <SelectItem value="face_follow">
                            <div>
                              <div className="font-medium">Face Follow</div>
                              <div className="text-xs text-neutral-400">Track and follow faces</div>
                            </div>
                          </SelectItem>
                          <SelectItem value="person_follow">
                            <div>
                              <div className="font-medium">Person Follow</div>
                              <div className="text-xs text-neutral-400">Detect and follow people</div>
                            </div>
                          </SelectItem>
                          <SelectItem value="aruco_track">
                            <div>
                              <div className="font-medium">ArUco Track</div>
                              <div className="text-xs text-neutral-400">Follow ArUco markers</div>
                            </div>
                          </SelectItem>
                          <SelectItem value="motion_detect">
                            <div>
                              <div className="font-medium">Motion Detect</div>
                              <div className="text-xs text-neutral-400">React to movement</div>
                            </div>
                          </SelectItem>
                          <SelectItem value="object_detect">
                            <div>
                              <div className="font-medium">Object Detect</div>
                              <div className="text-xs text-neutral-400">Detect any objects</div>
                            </div>
                          </SelectItem>
                        </SelectContent>
                      </Select>
                    </div>

                    <div className="space-y-3 pt-2 border-t border-neutral-800/50">
                      <div>
                        <div className="flex items-center justify-between mb-2">
                          <div className="flex items-center gap-1">
                            <span className="text-xs font-medium text-neutral-200">Confidence Threshold</span>
                            <div className="group relative">
                              <HelpCircle className="h-3 w-3 text-neutral-400 cursor-help" />
                              <div className="absolute left-0 top-4 w-56 bg-neutral-900 border border-neutral-700 rounded-md p-2 text-xs text-neutral-200 opacity-0 invisible group-hover:opacity-100 group-hover:visible transition-all z-50 shadow-lg">
                                How certain the detection must be (%). Higher = more accurate but fewer detections. Start with 70%.
                              </div>
                            </div>
                          </div>
                          <span className="text-xs font-bold text-neutral-100">{params.cvConfidenceThreshold}%</span>
                        </div>
                        <Slider
                          value={[params.cvConfidenceThreshold]}
                          onValueChange={(v: number[]) => setParam('cvConfidenceThreshold', v[0] ?? 0)}
                          min={50}
                          max={95}
                          step={5}
                          className="mt-2"
                          aria-label="Confidence Threshold"
                        />
                        <div className="flex justify-between text-[10px] text-neutral-500 mt-1">
                          <span>More Detections</span>
                          <span>More Accurate</span>
                        </div>
                      </div>

                      <div className="grid grid-cols-2 gap-3">
                        <NumberField 
                          label="Max Detections" 
                          value={params.cvMaxDetections} 
                          min={1} 
                          max={10} 
                          step={1}
                          onChange={(n) => setParam('cvMaxDetections', n)} 
                        />
                        <div className="grid gap-1">
                          <label className="text-xs text-neutral-300">Tracker Type</label>
                          <Select 
                            value={params.cvTrackerType} 
                            onValueChange={(v) => setParam('cvTrackerType', v as AutonomyParams['cvTrackerType'])}
                          >
                            <SelectTrigger className="bg-neutral-950 border-neutral-800 text-neutral-100 text-xs">
                              <SelectValue />
                            </SelectTrigger>
                            <SelectContent className="bg-neutral-950 border-neutral-800 text-neutral-100">
                              <SelectItem value="CSRT">CSRT (Best Accuracy)</SelectItem>
                              <SelectItem value="KCF">KCF (Fast)</SelectItem>
                              <SelectItem value="MIL">MIL (Balance)</SelectItem>
                            </SelectContent>
                          </Select>
                        </div>
                      </div>

                      <div className="space-y-2 pt-2 border-t border-neutral-800/50">
                        <ToggleRow
                          icon={<span className="inline-block h-4 w-4 rounded-sm bg-cyan-400/70" />}
                          label="Show Bounding Boxes"
                          description="Display detection boxes on camera feed"
                          checked={params.cvShowBoundingBoxes}
                          onCheckedChange={(v) => setParam('cvShowBoundingBoxes', v)}
                        />
                        <ToggleRow
                          icon={<Crosshair className="h-4 w-4 text-purple-400" />}
                          label="Object Tracking"
                          description="Smoothly track objects across frames"
                          checked={params.cvTrackObjects}
                          onCheckedChange={(v) => setParam('cvTrackObjects', v)}
                        />
                      </div>
                    </div>
                  </>
                )}
              </CardContent>
            </Card>

            {/* Quick Actions */}
            <Card className="bg-neutral-900/80 border-neutral-800">
              <CardContent className="p-3 grid gap-2">
                <div className="text-xs font-medium text-neutral-300">Quick Actions</div>
                <div className="flex gap-2">
                  <Button disabled={busy} variant="secondary" onClick={handleDock} className="gap-2 flex-1" aria-busy={busy}>
                    <Bot className="h-4 w-4" /> Return to Dock
                  </Button>
                </div>
                <p className="text-xs text-neutral-400">
                  Use "Return to Dock" to send your robot back to its charging/home position.
                </p>
              </CardContent>
            </Card>

            {/* ADVANCED toggle */}
            <button
              type="button"
              onClick={() => setShowAdvanced(v => !v)}
              className="text-left w-full rounded-md border border-neutral-800 bg-neutral-900/80 px-3 py-2 text-sm text-neutral-100 hover:bg-neutral-900 flex items-center justify-between transition-colors"
              aria-expanded={showAdvanced}
            >
              <div className="flex items-center gap-2">
                <Settings2 className="h-4 w-4 text-neutral-400" />
                <span>Advanced Settings</span>
              </div>
              <span className="text-neutral-400 text-xs">{showAdvanced ? 'Hide' : 'Show'} Expert Options</span>
            </button>

            {/* ADVANCED content */}
            {showAdvanced && (
              <div className="grid grid-cols-1 gap-3">
                {/* Navigation tuning */}
                <Card className="bg-neutral-900/80 border-neutral-800">
                  <CardContent className="p-3 grid gap-3">
                    <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
                      <NumberField label="Stop Distance (m)" value={params.avoidStopDistM} step={0.05} min={0.05} max={1.5}
                        onChange={(n) => setParam('avoidStopDistM', n)} />
                      <NumberField label="Line kP" value={params.line_kP} step={0.05} min={0} max={3}
                        onChange={(n) => setParam('line_kP', n)} />
                      <NumberField label="Search Yaw Rate (°/s)" value={params.searchYawRate} step={1} min={5} max={120}
                        onChange={(n) => setParam('searchYawRate', n)} />
                    </div>
                    <ToggleRow
                      icon={<span className="inline-block h-4 w-4 rounded-sm bg-neutral-400" />}
                      label="Lane Keeping (IR line hold)"
                      checked={params.laneKeeping}
                      onCheckedChange={(v) => setParam('laneKeeping', v)}
                    />
                  </CardContent>
                </Card>

                {/* Vision */}
                <Card className="bg-neutral-900/80 border-neutral-800">
                  <CardContent className="p-3 grid gap-3">
                    <p className="text-xs text-neutral-300">
                      Color/Object tracking uses HSV thresholds; steering centers the blob, speed scales by area.
                    </p>
                    <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
                      <TripletField
                        label="HSV Low"
                        value={params.hsvLow}
                        onChange={(arr) => setParam('hsvLow', arr as [number,number,number])}
                      />
                      <TripletField
                        label="HSV High"
                        value={params.hsvHigh}
                        onChange={(arr) => setParam('hsvHigh', arr as [number,number,number])}
                      />
                      <div className="grid grid-cols-2 gap-3">
                        <NumberField label="Min Area (px)" value={params.minArea} min={0} max={200000} step={100}
                          onChange={(n)=> setParam('minArea', n)} />
                        <NumberField label="Max Area (px)" value={params.maxArea} min={100} max={500000} step={100}
                          onChange={(n)=> setParam('maxArea', n)} />
                      </div>
                    </div>
                    <div className="text-[11px] text-neutral-400">
                      For ArUco/AprilTag/Face modes, backend should publish <code>{'{ id, dist, bearing }'}</code> or a face box; UI only sends <code>{'mode'}</code> + params.
                    </div>
                  </CardContent>
                </Card>

                {/* Behavior priorities */}
                <Card className="bg-neutral-900/80 border-neutral-800">
                  <CardContent className="p-3 grid gap-3">
                    <p className="text-xs text-neutral-300">Priority (top preempts lower). Use arrows to reorder.</p>
                    <PriorityEditor
                      items={params.priorities}
                      onChange={(items) => setParam('priorities', items)}
                    />
                  </CardContent>
                </Card>

                {/* Mapping + Safety */}
                <Card className="bg-neutral-900/80 border-neutral-800">
                  <CardContent className="p-3 grid gap-3">
                    <ToggleRow
                      icon={<span className="inline-block h-4 w-4 rounded-sm bg-neutral-400" />}
                      label="Enable Grid Mapping"
                      checked={params.gridEnabled}
                      onCheckedChange={(v) => setParam('gridEnabled', v)}
                    />
                    <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
                      <NumberField label="Cell Size (m)" value={params.gridCellSizeM} min={0.05} max={0.5} step={0.01}
                        onChange={(n)=> setParam('gridCellSizeM', n)} />
                      <NumberField label="Decay (0–1)" value={params.gridDecay} min={0} max={1} step={0.01}
                        onChange={(n)=> setParam('gridDecay', n)} />
                      <NumberField label="Battery Floor (%)" value={params.batteryMinPct} min={0} max={100} step={1}
                        onChange={(n)=> setParam('batteryMinPct', n)} />
                    </div>
                    <div className="text-[11px] text-neutral-400">
                      Safety overrides always win. Backend must stop if battery below floor, cliff detected, or ultrasonic &lt; stop distance.
                    </div>
                  </CardContent>
                </Card>

                {/* Presets */}
                <Card className="bg-neutral-900/80 border-neutral-800">
                  <CardContent className="p-3 grid gap-2">
                    <div className="flex flex-wrap gap-2">
                      <Button variant="secondary" className="gap-2" onClick={downloadPresets}>
                        <Save className="h-4 w-4" /> Export JSON
                      </Button>
                      <Button variant="secondary" className="gap-2" onClick={openFileDialog}>
                        <Upload className="h-4 w-4" /> Import JSON
                      </Button>
                      <input ref={fileRef} type="file" accept="application/json" className="hidden" onChange={onFileSelected} />
                    </div>
                    <div className="text-[11px] text-neutral-400">
                      Exports include mode, params, and waypoints (client-side preview). Import merges into current config.
                    </div>
                  </CardContent>
                </Card>

                <div className="flex items-center justify-end gap-2">
                  <Button disabled={busy || !onUpdate} onClick={handleApplyParams} variant="secondary" aria-busy={busy}>
                    Apply Params
                  </Button>
                </div>
              </div>
            )}

            {/* Footer tip */}
            <div className="text-[11px] text-neutral-300 flex items-start gap-2 flex-shrink-0">
              <span className="mt-0.5 inline-block h-1.5 w-1.5 rounded-full bg-neutral-500/70" />
              Hook these handlers to your WS/ROS bridge. The modal sends commands/params; stream telemetry elsewhere.
            </div>

            <div className="flex justify-end flex-shrink-0 sticky bottom-0 bg-neutral-950 pt-2 pb-2">
              <Button onClick={() => setOpen(false)}>Close</Button>
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  );
}

/* --------------------------------- Helpers --------------------------------- */

function canonicalizeMode(m: AutonomyMode): AutonomyMode {
  // Keep compatibility with the old "line_track" name
  if (m === 'line_track') return 'line_follow';
  return m;
}

function ToggleRow({
  icon, label, description, checked, onCheckedChange,
}: { icon: React.ReactNode; label: string; description?: string; checked: boolean; onCheckedChange: (v: boolean) => void }) {
  return (
    <div className="flex items-center justify-between rounded-lg border border-neutral-800 bg-neutral-950 px-3 py-2.5">
      <div className="flex-1">
        <div className="flex items-center gap-2 text-sm text-neutral-100">
          {icon} <span className="font-medium">{label}</span>
        </div>
        {description && (
          <div className="text-xs text-neutral-400 mt-0.5 ml-6">{description}</div>
        )}
      </div>
      <Switch checked={checked} onCheckedChange={onCheckedChange} aria-label={label} />
    </div>
  );
}

function SliderField({
  label, value, onChange,
}: { label: string; value: number; onChange: (n: number) => void }) {
  return (
    <div>
      <div className="flex items-center justify-between text-xs text-neutral-300">
        <span>{label}</span>
        <span className="tabular-nums text-neutral-100">{value}%</span>
      </div>
      <Slider
        value={[value]}
        onValueChange={(v: number[]) => onChange(v[0] ?? 0)}
        min={0}
        max={100}
        step={1}
        className="mt-2"
        aria-label={label}
      />
    </div>
  );
}

function NumberField({
  label, value, onChange, step = 0.1, min, max,
}: {
  label: string; value: number; onChange: (n: number) => void; step?: number; min?: number; max?: number;
}) {
  return (
    <div className="grid gap-1">
      <label className="text-xs text-neutral-300">{label}</label>
      <Input
        type="number"
        className="bg-neutral-950 border-neutral-800 text-neutral-100"
        value={Number.isFinite(value) ? value : 0}
        step={step}
        min={min}
        max={max}
        inputMode="decimal"
        onChange={(e) => {
          const n = Number(e.target.value);
          if (Number.isFinite(n)) onChange(clamp(n, min, max));
        }}
      />
    </div>
  );
}

function TripletField({
  label, value, onChange,
}: {
  label: string;
  value: [number,number,number];
  onChange: (arr: number[]) => void;
}) {
  const [h, s, v] = value;
  return (
    <div className="grid gap-1">
      <label className="text-xs text-neutral-300">{label}</label>
      <div className="grid grid-cols-3 gap-2">
        <Input type="number" value={h} min={0} max={179} step={1}
          className="bg-neutral-950 border-neutral-800 text-neutral-100"
          onChange={(e)=> onChange([clampNum(e.target.value, h, 0, 179), s, v])} />
        <Input type="number" value={s} min={0} max={255} step={1}
          className="bg-neutral-950 border-neutral-800 text-neutral-100"
          onChange={(e)=> onChange([h, clampNum(e.target.value, s, 0, 255), v])} />
        <Input type="number" value={v} min={0} max={255} step={1}
          className="bg-neutral-950 border-neutral-800 text-neutral-100"
          onChange={(e)=> onChange([h, s, clampNum(e.target.value, v, 0, 255)])} />
      </div>
    </div>
  );
}

function PriorityEditor({
  items, onChange,
}: {
  items: Array<'safety'|'avoid'|'edge'|'line'|'waypoints'|'vision'>;
  onChange: (items: Array<'safety'|'avoid'|'edge'|'line'|'waypoints'|'vision'>) => void;
}) {
  const move = (i: number, dir: -1 | 1) => {
    const j = i + dir;
    if (j < 0 || j >= items.length) return;
    const copy = items.slice();
    [copy[i], copy[j]] = [copy[j], copy[i]];
    onChange(copy);
  };
  return (
    <div className="grid gap-2">
      {items.map((k, i) => (
        <div key={`${k}-${i}`} className="flex items-center justify-between rounded border border-neutral-800 bg-neutral-950 px-2 py-1">
          <span className="text-sm capitalize text-neutral-100">{k}</span>
          <div className="flex gap-1">
            <Button variant="secondary" size="sm" onClick={() => move(i, -1)} disabled={i===0} aria-label={`Move ${k} up`}>↑</Button>
            <Button variant="secondary" size="sm" onClick={() => move(i, +1)} disabled={i===items.length-1} aria-label={`Move ${k} down`}>↓</Button>
          </div>
        </div>
      ))}
    </div>
  );
}

/* utils */
function clamp(n: number, min?: number, max?: number) {
  let v = n;
  if (typeof min === 'number') v = Math.max(min, v);
  if (typeof max === 'number') v = Math.min(max, v);
  return v;
}
function num(s: string, fallback: number) { const n = Number(s); return Number.isFinite(n) ? n : fallback; }
function clampNum(s: string, fallback: number, min: number, max: number) {
  return clamp(num(s, fallback), min, max);
}
function canonicalize(obj: any): any {
  // stable stringify (sort keys shallowly; arrays kept order)
  if (obj && typeof obj === 'object' && !Array.isArray(obj)) {
    return Object.keys(obj).sort().reduce((acc:any, k) => { acc[k] = canonicalize(obj[k]); return acc; }, {} as any);
  }
  if (Array.isArray(obj)) return obj.map(canonicalize);
  return obj;
}
function safeStableStringify(x: any) {
  try { return JSON.stringify(canonicalize(x)); } catch { return ''; }
}
