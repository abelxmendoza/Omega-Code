'use client';

/*
# File: /src/components/PerformanceDashboard.tsx
# Summary:
# Real-time performance monitoring dashboard.
# Polls /api/performance-proxy/metrics (30s) and /api/performance-proxy/cache (non-blocking).
# Circuit breaker: 3 failures → 60s backoff.
# Skips polling when backend is offline (from useSystemHealth).
*/

import React, { useState, useEffect, useRef, memo } from 'react';
import {
  Activity, Cpu, HardDrive, Wifi, Server, Shield,
  Thermometer, Battery, Zap, AlertTriangle, CheckCircle2,
  Clock, Radio,
} from 'lucide-react';
import { useSystemHealth } from '@/context/SystemHealthContext';

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
/* Constants                                                           */
/* ------------------------------------------------------------------ */

const POLL_MS      = 30_000;
const BACKOFF_MS   = 60_000;
const FAIL_THRESH  = 3;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

function statusColor(value: number, warn: number, crit: number) {
  if (value >= crit) return 'text-rose-400';
  if (value >= warn) return 'text-amber-400';
  return 'text-emerald-400';
}

function formatBytes(bytes: number) {
  const sizes = ['B', 'KB', 'MB', 'GB'];
  if (bytes === 0) return '0 B';
  const i = Math.floor(Math.log(bytes) / Math.log(1024));
  return `${(bytes / Math.pow(1024, i)).toFixed(1)} ${sizes[i]}`;
}

function formatUptime(seconds: number) {
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
  const health = useSystemHealth();
  const [perfData, setPerfData] = useState<PerformanceData | null>(null);
  const [cacheStats, setCacheStats] = useState<CacheStats | null>(null);
  const [monitoring, setMonitoring] = useState(false);

  const fetchRef = useRef<() => void>(() => {});

  useEffect(() => {
    if (!health.connected) {
      setMonitoring(false);
      return;
    }

    let mounted = true;
    let failCount = 0;
    let intervalId: ReturnType<typeof setInterval>;

    async function tick() {
      // Metrics (critical path)
      try {
        const res = await fetch('/api/performance-proxy/metrics');
        if (!mounted) return;
        if (res.ok) {
          failCount = 0;
          setPerfData(await res.json());
          setMonitoring(true);
          if (failCount === 0 && intervalId) {
            // recover to normal interval — reschedule handled below
          }
        } else {
          failCount++;
        }
      } catch {
        if (!mounted) return;
        failCount++;
      }

      // Cache stats (non-critical — never affects monitoring state)
      try {
        const res = await fetch('/api/performance-proxy/cache');
        if (mounted && res.ok) setCacheStats(await res.json());
      } catch { /* silent */ }

      // Circuit breaker escalation
      if (failCount >= FAIL_THRESH) {
        setMonitoring(false);
        clearInterval(intervalId);
        intervalId = setInterval(tick, BACKOFF_MS);
      }
    }

    fetchRef.current = tick;
    tick();
    intervalId = setInterval(tick, POLL_MS);

    return () => {
      mounted = false;
      clearInterval(intervalId);
    };
  }, [health.connected]);

  /* Loading / offline state */
  if (!perfData) {
    return (
      <div className="bg-gray-900 border border-white/10 rounded-lg overflow-hidden">
        <div className="flex items-center gap-2 px-4 py-2.5 bg-gray-800 border-b border-white/10">
          <Activity size={14} className="text-emerald-400" />
          <span className="text-sm font-bold tracking-wide text-white uppercase">Performance</span>
        </div>
        <div className="p-6 text-center text-sm text-white/40">
          {!health.connected
            ? 'Backend offline — waiting for connection'
            : 'Fetching metrics…'}
        </div>
      </div>
    );
  }

  /* Alert conditions */
  const alerts: { msg: string; level: 'critical' | 'warning' }[] = [];
  if (perfData.cpuUsage > 90)            alerts.push({ msg: 'High CPU usage', level: 'critical' });
  if (perfData.memoryPercent > 95)       alerts.push({ msg: 'High memory usage', level: 'critical' });
  if ((perfData.responseTime ?? 0) > 1000) alerts.push({ msg: 'Slow response time', level: 'critical' });
  if ((perfData.errorRate ?? 0) > 5)    alerts.push({ msg: 'High error rate', level: 'critical' });
  if (perfData.temperature && perfData.temperature > 70) alerts.push({ msg: 'CPU temperature critical', level: 'critical' });
  if (cacheStats && cacheStats.hitRate < 60) alerts.push({ msg: 'Low cache hit rate', level: 'warning' });

  return (
    <div className="bg-gray-900 border border-white/10 rounded-lg overflow-hidden">

      {/* Header */}
      <div className="flex items-center justify-between px-4 py-2.5 bg-gray-800 border-b border-white/10">
        <div className="flex items-center gap-2">
          <Activity size={14} className="text-emerald-400" />
          <span className="text-sm font-bold tracking-wide text-white uppercase">Performance</span>
          {perfData.piSpecific?.piModel && (
            <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 text-white/40 font-medium">
              {perfData.piSpecific.piModel}
            </span>
          )}
        </div>
        <div className="flex items-center gap-1.5">
          <span className={`w-1.5 h-1.5 rounded-full ${monitoring ? 'bg-emerald-400' : 'bg-rose-400'}`} />
          <span className="text-[11px] text-white/40">{monitoring ? 'Live' : 'Backoff'}</span>
        </div>
      </div>

      <div className="p-3 space-y-3">

        {/* Row 1: System + Network + Application */}
        <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">

          <Card>
            <SectionHeader icon={<Cpu size={11} />} label="System" colorClass="text-sky-400" />
            <Row
              label="CPU"
              value={`${perfData.cpuUsage.toFixed(1)}%`}
              valueClass={statusColor(perfData.cpuUsage, 70, 90)}
            />
            <Row
              label="Memory"
              value={`${perfData.memoryPercent.toFixed(1)}%`}
              valueClass={statusColor(perfData.memoryPercent, 80, 95)}
            />
            <Row
              label="Disk"
              value={`${perfData.diskUsage.toFixed(1)}%`}
              valueClass={statusColor(perfData.diskUsage, 85, 95)}
            />
            {perfData.temperature != null && (
              <Row
                label="Temp"
                value={`${perfData.temperature.toFixed(1)}°C`}
                valueClass={statusColor(perfData.temperature, 60, 70)}
              />
            )}
          </Card>

          <Card>
            <SectionHeader icon={<Wifi size={11} />} label="Network I/O" colorClass="text-emerald-400" />
            <Row label="Sent"      value={formatBytes(perfData.networkIO.bytesSent)} />
            <Row label="Received"  value={formatBytes(perfData.networkIO.bytesRecv)} />
            <Row label="WS Conns"  value={String(perfData.websocketConnections)} />
            {perfData.robotTelemetry?.network?.latency != null && (
              <Row label="Latency" value={`${perfData.robotTelemetry.network.latency}ms`} />
            )}
            {perfData.robotTelemetry?.network?.wifiSignal != null && (
              <Row
                label="Signal"
                value={`${perfData.robotTelemetry.network.wifiSignal}dBm`}
                valueClass={perfData.robotTelemetry.network.wifiSignal > -60 ? 'text-emerald-400' : 'text-amber-400'}
              />
            )}
          </Card>

          <Card>
            <SectionHeader icon={<Zap size={11} />} label="Application" colorClass="text-violet-400" />
            <Row
              label="Response"
              value={`${(perfData.responseTime ?? 0).toFixed(1)}ms`}
              valueClass={statusColor(perfData.responseTime ?? 0, 500, 1000)}
            />
            <Row
              label="Error Rate"
              value={`${(perfData.errorRate ?? 0).toFixed(2)}%`}
              valueClass={statusColor(perfData.errorRate ?? 0, 2, 5)}
            />
            <Row label="Throughput" value={`${(perfData.throughput ?? 0).toFixed(0)} req/s`} />
            <Row
              label="Uptime"
              value={perfData.uptime > 0 ? formatUptime(perfData.uptime) : '—'}
            />
          </Card>
        </div>

        {/* Row 2: Pi services + Power + Safety */}
        {(perfData.piSpecific || perfData.robotTelemetry) && (
          <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">

            {perfData.piSpecific && (
              <Card>
                <SectionHeader icon={<Server size={11} />} label="Pi Services" colorClass="text-amber-400" />
                <div className="flex flex-wrap gap-x-3 gap-y-1 mt-0.5">
                  {Object.entries(perfData.piSpecific.robotServices).map(([svc, on]) => (
                    <ServiceDot key={svc} on={on as boolean} label={svc} />
                  ))}
                </div>
                <div className="pt-1 space-y-1.5">
                  <Row
                    label="GPIO"
                    value={perfData.piSpecific.gpioStatus.status}
                    valueClass={perfData.piSpecific.gpioStatus.status === 'active' ? 'text-emerald-400' : 'text-rose-400'}
                  />
                  <Row
                    label="Camera"
                    value={perfData.piSpecific.cameraStatus.status}
                    valueClass={perfData.piSpecific.cameraStatus.enabled ? 'text-emerald-400' : 'text-rose-400'}
                  />
                </div>
              </Card>
            )}

            {perfData.robotTelemetry?.power && (
              <Card>
                <SectionHeader icon={<Battery size={11} />} label="Power" colorClass="text-yellow-400" />
                <Row label="Source" value={perfData.robotTelemetry.power.powerSource} />
                {perfData.robotTelemetry.power.voltage != null && (
                  <Row
                    label="Voltage"
                    value={`${perfData.robotTelemetry.power.voltage}V`}
                    valueClass={perfData.robotTelemetry.power.undervoltage ? 'text-rose-400' : 'text-emerald-400'}
                  />
                )}
                {perfData.robotTelemetry.power.powerConsumption != null && (
                  <Row label="Consumption" value={`${perfData.robotTelemetry.power.powerConsumption}W`} />
                )}
                {perfData.robotTelemetry.power.undervoltage && (
                  <div className="flex items-center gap-1 text-rose-400 text-[11px] font-semibold pt-0.5">
                    <AlertTriangle size={11} />
                    Undervoltage
                  </div>
                )}
              </Card>
            )}

            {perfData.robotTelemetry?.safety && (
              <Card>
                <SectionHeader icon={<Shield size={11} />} label="Safety" colorClass="text-rose-400" />
                <Row
                  label="E-Stop"
                  value={perfData.robotTelemetry.safety.emergencyStop ? 'ACTIVE' : 'Safe'}
                  valueClass={perfData.robotTelemetry.safety.emergencyStop ? 'text-rose-400' : 'text-emerald-400'}
                />
                <Row
                  label="Thermal"
                  value={perfData.robotTelemetry.safety.thermalProtection.overheated ? 'Overheated' : 'Normal'}
                  valueClass={perfData.robotTelemetry.safety.thermalProtection.overheated ? 'text-rose-400' : 'text-emerald-400'}
                />
                <Row
                  label="Battery"
                  value={perfData.robotTelemetry.safety.batteryProtection.lowBattery ? 'Low' : 'OK'}
                  valueClass={perfData.robotTelemetry.safety.batteryProtection.lowBattery ? 'text-amber-400' : 'text-emerald-400'}
                />
                <Row
                  label="Violations"
                  value={String(perfData.robotTelemetry.safety.safetyLimits.violations)}
                  valueClass={perfData.robotTelemetry.safety.safetyLimits.violations > 0 ? 'text-amber-400' : 'text-white'}
                />
              </Card>
            )}
          </div>
        )}

        {/* Row 3: Sensors + Motors + Cache */}
        <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">

          {perfData.robotTelemetry?.sensors && (
            <Card>
              <SectionHeader icon={<Radio size={11} />} label="Sensors" colorClass="text-sky-400" />
              {(Object.entries(perfData.robotTelemetry.sensors) as [string, { status: string; fps?: number | null; distance?: number | null; value?: number | null }][]).map(([name, s]) => {
                const display = s.fps != null ? `${s.fps}fps`
                  : s.distance != null ? `${s.distance}cm`
                  : s.value != null ? `${s.value}V`
                  : s.status;
                const ok = s.status === 'active' || s.status === 'connected';
                return (
                  <Row
                    key={name}
                    label={name.replace(/([A-Z])/g, ' $1').trim()}
                    value={display}
                    valueClass={ok ? 'text-emerald-400' : 'text-white/40'}
                  />
                );
              })}
            </Card>
          )}

          {perfData.robotTelemetry?.motors && (
            <Card>
              <SectionHeader icon={<Activity size={11} />} label="Motors" colorClass="text-purple-400" />
              <Row
                label="Left"
                value={perfData.robotTelemetry.motors.leftMotor.status}
                valueClass={perfData.robotTelemetry.motors.leftMotor.status === 'connected' ? 'text-emerald-400' : 'text-rose-400'}
              />
              <Row
                label="Right"
                value={perfData.robotTelemetry.motors.rightMotor.status}
                valueClass={perfData.robotTelemetry.motors.rightMotor.status === 'connected' ? 'text-emerald-400' : 'text-rose-400'}
              />
              <Row label="Servos" value={`${perfData.robotTelemetry.motors.servoMotors.length} active`} valueClass="text-sky-400" />
            </Card>
          )}

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
              <p className="text-xs text-white/30 italic">Unavailable</p>
            )}
          </Card>
        </div>

        {/* Alerts */}
        <div className="bg-gray-800/60 border border-white/8 rounded-lg p-3">
          <div className="flex items-center gap-1.5 text-xs font-bold uppercase tracking-wider mb-2 text-white/50">
            <AlertTriangle size={11} />
            Alerts
          </div>
          {alerts.length === 0 ? (
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
            Updated {new Date(perfData.timestamp).toLocaleTimeString()}
          </div>
          {perfData.source && <span>{perfData.source} · {perfData.deviceName}</span>}
        </div>

      </div>
    </div>
  );
});

PerformanceDashboard.displayName = 'PerformanceDashboard';

export default PerformanceDashboard;
