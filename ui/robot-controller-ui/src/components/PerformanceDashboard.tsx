'use client';

/*
# File: /src/components/PerformanceDashboard.tsx
# Summary:
# Real-time performance monitoring dashboard.
# Polls /api/performance-proxy/metrics every 30s regardless of backend status.
# Shows default placeholder values when backend is offline or data not yet loaded.
# Circuit breaker: 3 failures → 60s backoff, then retries.
*/

import React, { useState, useEffect, useRef, memo } from 'react';
import {
  Activity, Cpu, HardDrive, Wifi, Server, Shield,
  Thermometer, Battery, Zap, AlertTriangle, CheckCircle2,
  Clock, Radio,
} from 'lucide-react';
import { useCommand } from '@/context/CommandContext';

/* ------------------------------------------------------------------ */
/* Types                                                               */
/* ------------------------------------------------------------------ */

interface PerformanceData {
  timestamp: number;
  source: string;
  deviceName: string;
  cpuUsage: number;
  memoryUsage: number;
  memoryPercent: number;
  diskUsage: number;
  networkIO: {
    bytesSent: number;
    bytesRecv: number;
    packetsSent: number;
    packetsRecv: number;
  };
  websocketConnections: number;
  uptime: number;
  loadAverage: number;
  temperature: number | null;
  responseTime?: number;
  errorRate?: number;
  throughput?: number;
  piSpecific: {
    gpioStatus: { status: string; pins: string };
    cameraStatus: { status: string; enabled: boolean };
    robotServices: { movement: boolean; camera: boolean; sensors: boolean; lighting: boolean };
    piModel: string;
    firmwareVersion: string;
  };
  robotTelemetry: {
    power: {
      voltage: number | null;
      percentage: number | null;
      charging: boolean;
      powerSource: string;
      undervoltage: boolean;
      powerConsumption: number | null;
    };
    sensors: {
      camera: { fps: number | null; resolution: string | null; status: string };
      ultrasonic: { distance: number | null; status: string; pins: string };
      lineTracking: { sensors: number[]; status: string; pins: string };
      voltage: { value: number | null; status: string; adc: string; i2c: string };
      buzzer: { status: string; pin: string };
      leds: { status: string; pins: string };
    };
    motors: {
      leftMotor: { speed: number; position: number; temperature: number | null; status: string };
      rightMotor: { speed: number; position: number; temperature: number | null; status: string };
      servoMotors: { id: number; position: number; status: string }[];
      actuators: unknown[];
    };
    network: {
      wifiSignal: number | null;
      latency: number | null;
      throughput: number | null;
      connectionType: string;
      ipAddress: string | null;
    };
    autonomous: {
      mode: string;
      navigation: { status: string; target: unknown; path: unknown[] };
      obstacleAvoidance: { enabled: boolean; detected: boolean };
      lineFollowing: { enabled: boolean; onLine: boolean };
      mission: { active: boolean; progress: number };
    };
    safety: {
      emergencyStop: boolean;
      safetyLimits: { enabled: boolean; violations: number };
      collisionDetection: { enabled: boolean; detected: boolean };
      batteryProtection: { enabled: boolean; lowBattery: boolean };
      thermalProtection: { enabled: boolean; overheated: boolean };
    };
  };
}

interface CacheStats {
  hits: number;
  misses: number;
  hitRate: number;
  totalRequests: number;
}

/* ------------------------------------------------------------------ */
/* Default / offline placeholder data                                  */
/* ------------------------------------------------------------------ */

const DEFAULT_DATA: PerformanceData = {
  timestamp: 0,
  source: '—',
  deviceName: '—',
  cpuUsage: 0,
  memoryUsage: 0,
  memoryPercent: 0,
  diskUsage: 0,
  networkIO: { bytesSent: 0, bytesRecv: 0, packetsSent: 0, packetsRecv: 0 },
  websocketConnections: 0,
  uptime: 0,
  loadAverage: 0,
  temperature: null,
  responseTime: 0,
  errorRate: 0,
  throughput: 0,
  piSpecific: {
    gpioStatus: { status: '—', pins: '—' },
    cameraStatus: { status: '—', enabled: false },
    robotServices: { movement: false, camera: false, sensors: false, lighting: false },
    piModel: '—',
    firmwareVersion: '—',
  },
  robotTelemetry: {
    power: {
      voltage: null, percentage: null, charging: false,
      powerSource: '—', undervoltage: false, powerConsumption: null,
    },
    sensors: {
      camera:      { fps: null, resolution: null, status: '—' },
      ultrasonic:  { distance: null, status: '—', pins: '—' },
      lineTracking:{ sensors: [], status: '—', pins: '—' },
      voltage:     { value: null, status: '—', adc: '—', i2c: '—' },
      buzzer:      { status: '—', pin: '—' },
      leds:        { status: '—', pins: '—' },
    },
    motors: {
      leftMotor:  { speed: 0, position: 0, temperature: null, status: '—' },
      rightMotor: { speed: 0, position: 0, temperature: null, status: '—' },
      servoMotors: [],
      actuators: [],
    },
    network: { wifiSignal: null, latency: null, throughput: null, connectionType: '—', ipAddress: null },
    autonomous: {
      mode: '—',
      navigation: { status: '—', target: null, path: [] },
      obstacleAvoidance: { enabled: false, detected: false },
      lineFollowing: { enabled: false, onLine: false },
      mission: { active: false, progress: 0 },
    },
    safety: {
      emergencyStop: false,
      safetyLimits: { enabled: false, violations: 0 },
      collisionDetection: { enabled: false, detected: false },
      batteryProtection: { enabled: false, lowBattery: false },
      thermalProtection: { enabled: false, overheated: false },
    },
  },
};

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

const POLL_MS     = 30_000;
const BACKOFF_MS  = 60_000;
const FAIL_THRESH = 3;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

function statusColor(value: number, warn: number, crit: number) {
  if (value >= crit) return 'text-rose-400';
  if (value >= warn) return 'text-amber-400';
  return 'text-emerald-400';
}

function formatBytes(bytes: number) {
  if (!bytes) return '0 B';
  const sizes = ['B', 'KB', 'MB', 'GB'];
  const i = Math.floor(Math.log(bytes) / Math.log(1024));
  return `${(bytes / Math.pow(1024, i)).toFixed(1)} ${sizes[i]}`;
}

function formatUptime(seconds: number) {
  if (!seconds) return '—';
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = Math.floor(seconds % 60);
  if (h > 0) return `${h}h ${m}m`;
  if (m > 0) return `${m}m ${s}s`;
  return `${s}s`;
}

/* ------------------------------------------------------------------ */
/* Sub-components                                                      */
/* ------------------------------------------------------------------ */

function SectionHeader({ icon, label, colorClass }: {
  icon: React.ReactNode;
  label: string;
  colorClass: string;
}) {
  return (
    <div className={`flex items-center gap-1.5 text-xs font-bold uppercase tracking-wider mb-2 ${colorClass}`}>
      {icon}
      {label}
    </div>
  );
}

function Row({ label, value, valueClass = 'text-white' }: {
  label: string;
  value: React.ReactNode;
  valueClass?: string;
}) {
  return (
    <div className="flex justify-between items-center">
      <span className="text-white/50 text-xs">{label}</span>
      <span className={`text-xs font-semibold ${valueClass}`}>{value}</span>
    </div>
  );
}

function Card({ children }: { children: React.ReactNode }) {
  return (
    <div className="bg-gray-800/60 border border-white/8 rounded-lg p-3 space-y-1.5">
      {children}
    </div>
  );
}

function ServiceDot({ on, label }: { on: boolean; label: string }) {
  return (
    <div className="flex items-center gap-1">
      <span className={`w-1.5 h-1.5 rounded-full ${on ? 'bg-emerald-400' : 'bg-rose-400'}`} />
      <span className="text-[11px] capitalize text-white/60">{label}</span>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Component                                                           */
/* ------------------------------------------------------------------ */

const PerformanceDashboard: React.FC = memo(() => {
  const { status: wsStatus } = useCommand();
  const backendOnline = wsStatus === 'connected';

  const [perfData, setPerfData] = useState<PerformanceData>(DEFAULT_DATA);
  const [cacheStats, setCacheStats] = useState<CacheStats | null>(null);
  const [live, setLive] = useState(false);
  const [lastFetched, setLastFetched] = useState<number | null>(null);

  useEffect(() => {
    let mounted = true;
    let failCount = 0;
    let intervalId: ReturnType<typeof setInterval>;

    async function tick() {
      try {
        const res = await fetch('/api/performance-proxy/metrics');
        if (!mounted) return;
        if (res.ok) {
          const raw = await res.json();
          failCount = 0;
          // Merge over DEFAULT_DATA so missing backend fields never crash
          setPerfData({
            ...DEFAULT_DATA,
            ...raw,
            networkIO:      { ...DEFAULT_DATA.networkIO,      ...(raw.networkIO      ?? {}) },
            piSpecific:     { ...DEFAULT_DATA.piSpecific,     ...(raw.piSpecific     ?? {}),
              gpioStatus:     { ...DEFAULT_DATA.piSpecific.gpioStatus,     ...(raw.piSpecific?.gpioStatus     ?? {}) },
              cameraStatus:   { ...DEFAULT_DATA.piSpecific.cameraStatus,   ...(raw.piSpecific?.cameraStatus   ?? {}) },
              robotServices:  { ...DEFAULT_DATA.piSpecific.robotServices,  ...(raw.piSpecific?.robotServices  ?? {}) },
            },
            robotTelemetry: {
              ...DEFAULT_DATA.robotTelemetry,
              ...(raw.robotTelemetry ?? {}),
              power:      { ...DEFAULT_DATA.robotTelemetry.power,      ...(raw.robotTelemetry?.power      ?? {}) },
              sensors:    { ...DEFAULT_DATA.robotTelemetry.sensors,    ...(raw.robotTelemetry?.sensors    ?? {}) },
              motors:     { ...DEFAULT_DATA.robotTelemetry.motors,     ...(raw.robotTelemetry?.motors     ?? {}) },
              network:    { ...DEFAULT_DATA.robotTelemetry.network,    ...(raw.robotTelemetry?.network    ?? {}) },
              autonomous: { ...DEFAULT_DATA.robotTelemetry.autonomous, ...(raw.robotTelemetry?.autonomous ?? {}) },
              safety:     { ...DEFAULT_DATA.robotTelemetry.safety,     ...(raw.robotTelemetry?.safety     ?? {}) },
            },
          });
          setLive(true);
          setLastFetched(Date.now());
          // Reset to normal interval on recovery
          if (intervalId) {
            clearInterval(intervalId);
            intervalId = setInterval(tick, POLL_MS);
          }
        } else {
          failCount++;
        }
      } catch {
        if (!mounted) return;
        failCount++;
      }

      // Cache stats — never affects live state
      try {
        const res = await fetch('/api/performance-proxy/cache');
        if (mounted && res.ok) setCacheStats(await res.json());
      } catch { /* silent */ }

      // Circuit breaker
      if (failCount >= FAIL_THRESH) {
        setLive(false);
        clearInterval(intervalId);
        intervalId = setInterval(tick, BACKOFF_MS);
      }
    }

    tick();
    intervalId = setInterval(tick, POLL_MS);

    return () => {
      mounted = false;
      clearInterval(intervalId);
    };
  }, []); // runs forever — no health gate

  /* Alert conditions — only when live */
  const alerts: { msg: string; level: 'critical' | 'warning' }[] = [];
  if (live) {
    if (perfData.cpuUsage > 90)                   alerts.push({ msg: 'High CPU usage',        level: 'critical' });
    if (perfData.memoryPercent > 95)              alerts.push({ msg: 'High memory usage',     level: 'critical' });
    if ((perfData.responseTime ?? 0) > 1000)      alerts.push({ msg: 'Slow response time',    level: 'critical' });
    if ((perfData.errorRate ?? 0) > 5)            alerts.push({ msg: 'High error rate',       level: 'critical' });
    if (perfData.temperature && perfData.temperature > 70) alerts.push({ msg: 'CPU temperature critical', level: 'critical' });
    if (cacheStats && cacheStats.hitRate < 60)    alerts.push({ msg: 'Low cache hit rate',    level: 'warning'  });
  }

  const headerStatus = live ? 'Live' : backendOnline ? 'Fetching…' : 'Offline';
  const headerDot    = live ? 'bg-emerald-400' : backendOnline ? 'bg-amber-400 animate-pulse' : 'bg-rose-400';

  return (
    <div className="bg-gray-900 border border-white/10 rounded-lg overflow-hidden">

      {/* Header */}
      <div className="flex items-center justify-between px-4 py-2.5 bg-gray-800 border-b border-white/10">
        <div className="flex items-center gap-2">
          <Activity size={14} className="text-emerald-400" />
          <span className="text-sm font-bold tracking-wide text-white uppercase">Performance</span>
          {live && perfData.piSpecific?.piModel && perfData.piSpecific.piModel !== '—' && (
            <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 text-white/40 font-medium">
              {perfData.piSpecific.piModel}
            </span>
          )}
        </div>
        <div className="flex items-center gap-1.5">
          <span className={`w-1.5 h-1.5 rounded-full ${headerDot}`} />
          <span className="text-[11px] text-white/40">{headerStatus}</span>
        </div>
      </div>

      <div className="p-3 space-y-3">

        {/* Row 1: System + Network + Application */}
        <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">

          <Card>
            <SectionHeader icon={<Cpu size={11} />} label="System" colorClass="text-sky-400" />
            <Row
              label="CPU"
              value={live ? `${perfData.cpuUsage.toFixed(1)}%` : '—'}
              valueClass={live ? statusColor(perfData.cpuUsage, 70, 90) : 'text-white/30'}
            />
            <Row
              label="Memory"
              value={live ? `${perfData.memoryPercent.toFixed(1)}%` : '—'}
              valueClass={live ? statusColor(perfData.memoryPercent, 80, 95) : 'text-white/30'}
            />
            <Row
              label="Disk"
              value={live ? `${perfData.diskUsage.toFixed(1)}%` : '—'}
              valueClass={live ? statusColor(perfData.diskUsage, 85, 95) : 'text-white/30'}
            />
            <Row
              label="Temp"
              value={live && perfData.temperature != null ? `${perfData.temperature.toFixed(1)}°C` : '—'}
              valueClass={live && perfData.temperature != null ? statusColor(perfData.temperature, 60, 70) : 'text-white/30'}
            />
          </Card>

          <Card>
            <SectionHeader icon={<Wifi size={11} />} label="Network I/O" colorClass="text-emerald-400" />
            <Row label="Sent"     value={live ? formatBytes(perfData.networkIO.bytesSent) : '—'} valueClass={live ? 'text-white' : 'text-white/30'} />
            <Row label="Received" value={live ? formatBytes(perfData.networkIO.bytesRecv) : '—'} valueClass={live ? 'text-white' : 'text-white/30'} />
            <Row label="WS Conns" value={live ? String(perfData.websocketConnections) : '—'}     valueClass={live ? 'text-white' : 'text-white/30'} />
            <Row
              label="Latency"
              value={live && perfData.robotTelemetry?.network?.latency != null ? `${perfData.robotTelemetry.network.latency}ms` : '—'}
              valueClass={live ? 'text-white' : 'text-white/30'}
            />
          </Card>

          <Card>
            <SectionHeader icon={<Zap size={11} />} label="Application" colorClass="text-violet-400" />
            <Row
              label="Response"
              value={live ? `${(perfData.responseTime ?? 0).toFixed(1)}ms` : '—'}
              valueClass={live ? statusColor(perfData.responseTime ?? 0, 500, 1000) : 'text-white/30'}
            />
            <Row
              label="Error Rate"
              value={live ? `${(perfData.errorRate ?? 0).toFixed(2)}%` : '—'}
              valueClass={live ? statusColor(perfData.errorRate ?? 0, 2, 5) : 'text-white/30'}
            />
            <Row label="Throughput" value={live ? `${(perfData.throughput ?? 0).toFixed(0)} req/s` : '—'} valueClass={live ? 'text-white' : 'text-white/30'} />
            <Row label="Uptime"     value={live ? formatUptime(perfData.uptime) : '—'}                    valueClass={live ? 'text-white' : 'text-white/30'} />
          </Card>
        </div>

        {/* Row 2: Pi services + Power + Safety */}
        <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">

          <Card>
            <SectionHeader icon={<Server size={11} />} label="Pi Services" colorClass="text-amber-400" />
            <div className="flex flex-wrap gap-x-3 gap-y-1 mt-0.5">
              {Object.entries(perfData.piSpecific.robotServices).map(([svc, on]) => (
                <ServiceDot key={svc} on={live ? (on as boolean) : false} label={svc} />
              ))}
            </div>
            <div className="pt-1 space-y-1.5">
              <Row
                label="GPIO"
                value={live ? perfData.piSpecific.gpioStatus.status : '—'}
                valueClass={live && perfData.piSpecific.gpioStatus.status === 'active' ? 'text-emerald-400' : live ? 'text-rose-400' : 'text-white/30'}
              />
              <Row
                label="Camera"
                value={live ? perfData.piSpecific.cameraStatus.status : '—'}
                valueClass={live && perfData.piSpecific.cameraStatus.enabled ? 'text-emerald-400' : live ? 'text-rose-400' : 'text-white/30'}
              />
            </div>
          </Card>

          <Card>
            <SectionHeader icon={<Battery size={11} />} label="Power" colorClass="text-yellow-400" />
            <Row label="Source"  value={live ? perfData.robotTelemetry.power.powerSource : '—'} valueClass={live ? 'text-white' : 'text-white/30'} />
            <Row
              label="Voltage"
              value={live && perfData.robotTelemetry.power.voltage != null ? `${perfData.robotTelemetry.power.voltage}V` : '—'}
              valueClass={live ? (perfData.robotTelemetry.power.undervoltage ? 'text-rose-400' : 'text-emerald-400') : 'text-white/30'}
            />
            <Row
              label="Consumption"
              value={live && perfData.robotTelemetry.power.powerConsumption != null ? `${perfData.robotTelemetry.power.powerConsumption}W` : '—'}
              valueClass={live ? 'text-white' : 'text-white/30'}
            />
            {live && perfData.robotTelemetry.power.undervoltage && (
              <div className="flex items-center gap-1 text-rose-400 text-[11px] font-semibold pt-0.5">
                <AlertTriangle size={11} />
                Undervoltage
              </div>
            )}
          </Card>

          <Card>
            <SectionHeader icon={<Shield size={11} />} label="Safety" colorClass="text-rose-400" />
            <Row
              label="E-Stop"
              value={live ? (perfData.robotTelemetry.safety.emergencyStop ? 'ACTIVE' : 'Safe') : '—'}
              valueClass={live ? (perfData.robotTelemetry.safety.emergencyStop ? 'text-rose-400' : 'text-emerald-400') : 'text-white/30'}
            />
            <Row
              label="Thermal"
              value={live ? (perfData.robotTelemetry.safety.thermalProtection.overheated ? 'Overheated' : 'Normal') : '—'}
              valueClass={live ? (perfData.robotTelemetry.safety.thermalProtection.overheated ? 'text-rose-400' : 'text-emerald-400') : 'text-white/30'}
            />
            <Row
              label="Battery"
              value={live ? (perfData.robotTelemetry.safety.batteryProtection.lowBattery ? 'Low' : 'OK') : '—'}
              valueClass={live ? (perfData.robotTelemetry.safety.batteryProtection.lowBattery ? 'text-amber-400' : 'text-emerald-400') : 'text-white/30'}
            />
            <Row
              label="Violations"
              value={live ? String(perfData.robotTelemetry.safety.safetyLimits.violations) : '—'}
              valueClass={live ? (perfData.robotTelemetry.safety.safetyLimits.violations > 0 ? 'text-amber-400' : 'text-white') : 'text-white/30'}
            />
          </Card>
        </div>

        {/* Row 3: Sensors + Motors + Cache */}
        <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">

          <Card>
            <SectionHeader icon={<Radio size={11} />} label="Sensors" colorClass="text-sky-400" />
            {(Object.entries(perfData.robotTelemetry.sensors) as [string, { status: string; fps?: number | null; distance?: number | null; value?: number | null }][]).map(([name, s]) => {
              const display = !live ? '—'
                : s.fps != null      ? `${s.fps}fps`
                : s.distance != null ? `${s.distance}cm`
                : s.value != null    ? `${s.value}V`
                : s.status;
              const ok = live && (s.status === 'active' || s.status === 'connected');
              return (
                <Row
                  key={name}
                  label={name.replace(/([A-Z])/g, ' $1').trim()}
                  value={display}
                  valueClass={ok ? 'text-emerald-400' : 'text-white/30'}
                />
              );
            })}
          </Card>

          <Card>
            <SectionHeader icon={<Activity size={11} />} label="Motors" colorClass="text-purple-400" />
            <Row
              label="Left"
              value={live ? perfData.robotTelemetry.motors.leftMotor.status : '—'}
              valueClass={live && perfData.robotTelemetry.motors.leftMotor.status === 'connected' ? 'text-emerald-400' : live ? 'text-rose-400' : 'text-white/30'}
            />
            <Row
              label="Right"
              value={live ? perfData.robotTelemetry.motors.rightMotor.status : '—'}
              valueClass={live && perfData.robotTelemetry.motors.rightMotor.status === 'connected' ? 'text-emerald-400' : live ? 'text-rose-400' : 'text-white/30'}
            />
            <Row
              label="Servos"
              value={live ? `${perfData.robotTelemetry.motors.servoMotors.length} active` : '—'}
              valueClass={live ? 'text-sky-400' : 'text-white/30'}
            />
          </Card>

          <Card>
            <SectionHeader icon={<HardDrive size={11} />} label="Cache" colorClass="text-amber-400" />
            {cacheStats ? (
              <>
                <Row
                  label="Hit Rate"
                  value={`${cacheStats.hitRate.toFixed(1)}%`}
                  valueClass={statusColor(100 - cacheStats.hitRate, 20, 40)}
                />
                <Row label="Total Reqs" value={String(cacheStats.totalRequests)} />
                <Row label="Hits"       value={String(cacheStats.hits)} />
                <Row label="Misses"     value={String(cacheStats.misses)} />
              </>
            ) : (
              <p className="text-xs text-white/30 italic">—</p>
            )}
          </Card>
        </div>

        {/* Alerts */}
        <div className="bg-gray-800/60 border border-white/8 rounded-lg p-3">
          <div className="flex items-center gap-1.5 text-xs font-bold uppercase tracking-wider mb-2 text-white/50">
            <AlertTriangle size={11} />
            Alerts
          </div>
          {!live ? (
            <div className="flex items-center gap-1.5 text-xs text-white/30 italic">
              Awaiting data…
            </div>
          ) : alerts.length === 0 ? (
            <div className="flex items-center gap-1.5 text-xs text-emerald-400 font-semibold">
              <CheckCircle2 size={12} />
              All systems normal
            </div>
          ) : (
            <div className="space-y-1">
              {alerts.map((a, i) => (
                <div key={i} className={`flex items-center gap-1.5 text-xs font-semibold ${a.level === 'critical' ? 'text-rose-400' : 'text-amber-400'}`}>
                  <AlertTriangle size={11} />
                  {a.msg}
                </div>
              ))}
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="flex items-center justify-between text-[11px] text-white/25 px-0.5">
          <div className="flex items-center gap-1">
            <Clock size={10} />
            {lastFetched ? `Updated ${new Date(lastFetched).toLocaleTimeString()}` : 'Not yet fetched'}
          </div>
          {live && perfData.source !== '—' && (
            <span>{perfData.source} · {perfData.deviceName}</span>
          )}
        </div>

      </div>
    </div>
  );
});

PerformanceDashboard.displayName = 'PerformanceDashboard';

export default PerformanceDashboard;
