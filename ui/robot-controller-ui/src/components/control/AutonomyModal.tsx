/*
# File: /src/components/control/AutonomyModal.tsx
# Summary:
#   Modal autonomy controls with improved contrast + a vibrant trigger button.
*/

'use client';

import React, { useMemo, useState } from 'react';
import {
  Bot, Play, Square, Gauge, Shield, Radar, Zap, Flag, Crosshair, Settings2,
} from 'lucide-react';

import { Button } from '@/components/ui/button';
import { Dialog, DialogContent, DialogHeader, DialogTitle, DialogTrigger } from '@/components/ui/dialog';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select';
import { Switch } from '@/components/ui/switch';
import { Slider } from '@/components/ui/slider';
import { Input } from '@/components/ui/input';
import { Badge } from '@/components/ui/badge';
import { Card, CardContent } from '@/components/ui/card';

export type AutonomyParams = {
  speedPct: number;
  aggressiveness: number;
  obstacleAvoidance: boolean;
  laneKeeping: boolean;
  headlights: boolean;
};

export type AutonomyMode = 'idle' | 'patrol' | 'follow' | 'dock' | 'line_track';

export type AutonomyModalProps = {
  className?: string;
  initialMode?: AutonomyMode;
  initialParams?: Partial<AutonomyParams>;
  onStart?: (mode: AutonomyMode, params: AutonomyParams) => Promise<void> | void;
  onStop?: () => Promise<void> | void;
  onDock?: () => Promise<void> | void;
  onSetWaypoint?: (label: string, lat: number, lon: number) => Promise<void> | void;
  connected?: boolean;
  autonomyActive?: boolean;
  batteryPct?: number;
  triggerLabel?: string;
};

const defaults: AutonomyParams = {
  speedPct: 40,
  aggressiveness: 30,
  obstacleAvoidance: true,
  laneKeeping: true,
  headlights: false,
};

export default function AutonomyModal({
  className,
  initialMode = 'idle',
  initialParams = {},
  onStart,
  onStop,
  onDock,
  onSetWaypoint,
  connected = true,
  autonomyActive = false,
  batteryPct = 100,
  triggerLabel = 'Autonomy',
}: AutonomyModalProps) {
  const [open, setOpen] = useState(false);
  const [mode, setMode] = useState<AutonomyMode>(initialMode);
  const [params, setParams] = useState<AutonomyParams>({ ...defaults, ...initialParams });
  const [wpLabel, setWpLabel] = useState('Home');
  const [wpLat, setWpLat] = useState<string>('');
  const [wpLon, setWpLon] = useState<string>('');
  const [busy, setBusy] = useState(false);

  const statusTone = useMemo(() => {
    if (!connected) return 'bg-red-500/15 text-red-400 border-red-500/40';
    if (autonomyActive) return 'bg-emerald-500/15 text-emerald-300 border-emerald-500/40';
    return 'bg-amber-500/15 text-amber-300 border-amber-500/40';
  }, [connected, autonomyActive]);

  async function handleStart() { setBusy(true); try { await onStart?.(mode, params); } finally { setBusy(false); } }
  async function handleStop()  { setBusy(true); try { await onStop?.(); }            finally { setBusy(false); } }
  async function handleDock()  { setBusy(true); try { setMode('dock'); await onDock?.(); } finally { setBusy(false); } }
  async function handleSetWaypoint() {
    if (!wpLabel || !wpLat || !wpLon) return;
    const lat = Number(wpLat), lon = Number(wpLon);
    if (Number.isNaN(lat) || Number.isNaN(lon)) return;
    setBusy(true);
    try { await onSetWaypoint?.(wpLabel, lat, lon); } finally { setBusy(false); }
  }

  return (
    <div className={className}>
      <Dialog open={open} onOpenChange={setOpen}>
        <DialogTrigger asChild>
          {/* 💥 Vibrant button that stands out */}
          <Button
            className="
              mt-3 gap-2 border-0
              bg-gradient-to-r from-emerald-600 via-teal-600 to-cyan-600
              hover:from-emerald-500 hover:via-teal-500 hover:to-cyan-500
              text-white shadow-lg shadow-emerald-500/25
              focus-visible:ring-2 focus-visible:ring-emerald-400/60
            "
          >
            <Bot className="h-4 w-4" /> {triggerLabel}
          </Button>
        </DialogTrigger>

        {/* 🌓 Higher-contrast modal */}
        <DialogContent className="sm:max-w-xl p-0 overflow-hidden border border-neutral-700 bg-neutral-900 text-neutral-100">
          <DialogHeader className="px-5 pt-5">
            <DialogTitle className="flex items-center gap-2 text-lg">
              <Settings2 className="h-5 w-5" /> Autonomy Controls
            </DialogTitle>
          </DialogHeader>

          {/* Status Row */}
          <div className="px-5 pb-3 flex items-center justify-between gap-2">
            <Badge variant="outline" className={`${statusTone} border`}>
              {connected ? (autonomyActive ? 'Active' : 'Connected') : 'Disconnected'}
            </Badge>
            <div className="flex items-center gap-2 text-xs text-neutral-200">
              <Gauge className="h-4 w-4" /> {batteryPct}%
            </div>
          </div>

          <div className="px-5 pb-5 grid grid-cols-1 gap-4">
            {/* Mode + Start/Stop */}
            <Card className="bg-neutral-800 border-neutral-700">
              <CardContent className="p-4 grid gap-4">
                <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 items-end">
                  <div>
                    <label className="text-xs text-neutral-200">Mode</label>
                    <Select value={mode} onValueChange={(v) => setMode(v as AutonomyMode)}>
                      <SelectTrigger className="mt-1 bg-neutral-900 border-neutral-700 text-neutral-100">
                        <SelectValue placeholder="Select mode" />
                      </SelectTrigger>
                      <SelectContent className="bg-neutral-900 border-neutral-700 text-neutral-100">
                        <SelectItem value="idle">Idle</SelectItem>
                        <SelectItem value="patrol">Patrol</SelectItem>
                        <SelectItem value="follow">Follow</SelectItem>
                        <SelectItem value="line_track">Line Track</SelectItem>
                        <SelectItem value="dock">Dock</SelectItem>
                      </SelectContent>
                    </Select>
                  </div>
                  <div className="flex gap-2">
                    <Button disabled={busy} onClick={handleStart} className="gap-2">
                      <Play className="h-4 w-4" /> Start
                    </Button>
                    <Button disabled={busy} variant="destructive" onClick={handleStop} className="gap-2">
                      <Square className="h-4 w-4" /> Stop
                    </Button>
                  </div>
                </div>

                {/* Safety & Behaviors */}
                <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
                  <ToggleRow
                    icon={<Shield className="h-4 w-4" />}
                    label="Obstacle Avoidance"
                    checked={params.obstacleAvoidance}
                    onCheckedChange={(v) => setParams((p) => ({ ...p, obstacleAvoidance: v }))}
                  />
                  <ToggleRow
                    icon={<Radar className="h-4 w-4" />}
                    label="Lane Keeping"
                    checked={params.laneKeeping}
                    onCheckedChange={(v) => setParams((p) => ({ ...p, laneKeeping: v }))}
                  />
                  <ToggleRow
                    icon={<Zap className="h-4 w-4" />}
                    label="Headlights"
                    checked={params.headlights}
                    onCheckedChange={(v) => setParams((p) => ({ ...p, headlights: v }))}
                  />
                </div>

                {/* Tunables */}
                <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
                  <SliderField
                    label="Speed"
                    value={params.speedPct}
                    onChange={(n) => setParams((p) => ({ ...p, speedPct: n }))}
                  />
                  <SliderField
                    label="Aggressiveness"
                    value={params.aggressiveness}
                    onChange={(n) => setParams((p) => ({ ...p, aggressiveness: n }))}
                  />
                </div>
              </CardContent>
            </Card>

            {/* Waypoints */}
            <Card className="bg-neutral-800 border-neutral-700">
              <CardContent className="p-4 grid gap-3">
                <div className="flex items-center gap-2 text-sm font-medium text-neutral-100">
                  <Flag className="h-4 w-4" /> Waypoint
                </div>
                <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
                  <Input value={wpLabel} onChange={(e) => setWpLabel(e.target.value)} placeholder="Label" className="bg-neutral-900 border-neutral-700 placeholder:text-neutral-400" />
                  <Input value={wpLat}   onChange={(e) => setWpLat(e.target.value)}   placeholder="Lat"   className="bg-neutral-900 border-neutral-700 placeholder:text-neutral-400" />
                  <Input value={wpLon}   onChange={(e) => setWpLon(e.target.value)}   placeholder="Lon"   className="bg-neutral-900 border-neutral-700 placeholder:text-neutral-400" />
                </div>
                <div className="flex gap-2">
                  <Button disabled={busy || !wpLat || !wpLon} onClick={handleSetWaypoint} className="gap-2">
                    <Crosshair className="h-4 w-4" /> Set Waypoint
                  </Button>
                  <Button disabled={busy} variant="secondary" onClick={handleDock} className="gap-2">
                    <Bot className="h-4 w-4" /> Dock
                  </Button>
                </div>
              </CardContent>
            </Card>

            {/* Tip */}
            <div className="text-[11px] text-neutral-200/80 flex items-start gap-2">
              <span className="mt-0.5 inline-block h-1.5 w-1.5 rounded-full bg-neutral-400/70" />
              Wire these handlers to your API/ROS bridge. Keep modal push-only; stream telemetry elsewhere.
            </div>
          </div>
        </DialogContent>
      </Dialog>
    </div>
  );
}

/* ——— helpers ——— */
function ToggleRow({
  icon, label, checked, onCheckedChange,
}: { icon: React.ReactNode; label: string; checked: boolean; onCheckedChange: (v: boolean) => void }) {
  return (
    <div className="flex items-center justify-between rounded-xl border border-neutral-700 bg-neutral-900 px-3 py-2">
      <div className="flex items-center gap-2 text-sm text-neutral-100">{icon} <span>{label}</span></div>
      <Switch checked={checked} onCheckedChange={onCheckedChange} />
    </div>
  );
}

function SliderField({ label, value, onChange }: { label: string; value: number; onChange: (n: number) => void }) {
  return (
    <div>
      <div className="flex items-center justify-between text-xs text-neutral-200">
        <span>{label}</span>
        <span className="tabular-nums text-neutral-100">{value}%</span>
      </div>
      <Slider
        value={[value]}
        onValueChange={(v) => onChange(v[0] ?? 0)}
        min={0}
        max={100}
        step={1}
        className="mt-2"
      />
    </div>
  );
}
