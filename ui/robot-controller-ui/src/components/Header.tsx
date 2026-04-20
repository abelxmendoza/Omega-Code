/*
# File: /Omega-Code/ui/robot-controller-ui/src/components/Header.tsx
# Summary:
#   App header with overall health + battery and per-service status pills.
#   - Profile-aware endpoints (via netProfile)
#   - WS services: ping→pong for latency (movement/line/lighting)
#   - Ultrasonic: any message marks alive (no pong)
#   - Video: probes sibling `/health` (cheap 200/503) instead of MJPEG
#   - Live Link pill: toggles Network Wizard + exposes Quick Actions
#   - Polls /api/net/summary every 8s with timeout + abort + stale-guard
#   - Shows current profile and MOCK badge (when NEXT_PUBLIC_MOCK_WS=1|true)
#   - Defensive error handling throughout; never throws during render
*/

'use client';

import React, { useMemo, useState, useEffect, useCallback, useRef } from 'react';
import Link from 'next/link';
import { useRouter } from 'next/router';
import { CheckCircle, XCircle, Wifi, SlidersHorizontal, Download, Activity } from 'lucide-react';
import { useWsStatus, ServiceStatus } from '../hooks/useWsStatus';
import { useHttpStatus, HttpStatus } from '../hooks/useHttpStatus';
import { net } from '@/utils/netProfile';
import { toHealthUrl } from '@/utils/urlHelpers';
import { CapabilityInfoModal } from './capability/CapabilityInfoModal';
import { useServiceSafety } from '@/hooks/useServiceSafety';
import ConnectionModeBadge from './ConnectionModeBadge';

// Install button component for PWA
const InstallButton: React.FC = () => {
  const [deferredPrompt, setDeferredPrompt] = useState<any>(null);
  const [isInstalled, setIsInstalled] = useState(false);

  useEffect(() => {
    // Only enable install button in production
    if (process.env.NODE_ENV !== 'production') {
      return;
    }

    // Check if already installed
    if (window.matchMedia('(display-mode: standalone)').matches || (window.navigator as any).standalone) {
      setIsInstalled(true);
      return;
    }

    const handleBeforeInstallPrompt = (e: Event) => {
      e.preventDefault();
      setDeferredPrompt(e);
    };

    const handleAppInstalled = () => {
      setIsInstalled(true);
      setDeferredPrompt(null);
    };

    window.addEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
    window.addEventListener('appinstalled', handleAppInstalled);

    return () => {
      window.removeEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
      window.removeEventListener('appinstalled', handleAppInstalled);
    };
  }, []);

  const handleInstall = async () => {
    if (!deferredPrompt) return;
    deferredPrompt.prompt();
    const { outcome } = await deferredPrompt.userChoice;
    if (outcome === 'accepted') {
      setDeferredPrompt(null);
    }
  };

  if (isInstalled || !deferredPrompt) return null;

  return (
    <div className="text-[10px] px-1.5 py-0.5 rounded bg-purple-500/20 border border-purple-400/40 text-purple-100 hover:bg-purple-500/30 transition-colors cursor-pointer">
      <button 
        onClick={handleInstall}
        title="Install App"
        className="flex items-center gap-1"
      >
        <Download className="w-3 h-3" />
      </button>
    </div>
  );
};

interface HeaderProps {
  batteryLevel: number | null;
  /** Pass current gamepad connected state from useGamepad hook */
  gamepadConnected?: boolean;
  /** Name of the connected gamepad (for tooltip) */
  gamepadName?: string;
  /** True when gamepad is connected but sending is paused */
  gamepadPaused?: boolean;
}

/* ----------------------------- utilities ------------------------------ */

type HeaderState = ServiceStatus | HttpStatus; // includes 'no_camera'

const truthy = (v: unknown) =>
  String(v ?? '').trim().toLowerCase() === '1' ||
  String(v ?? '').trim().toLowerCase() === 'true' ||
  String(v ?? '').trim().toLowerCase() === 'yes';

const MOCK_WS = truthy(process.env.NEXT_PUBLIC_MOCK_WS);

const getActiveProfile = (): 'lan' | 'tailscale' | 'local' => {
  // Only check URL parameters on client-side to avoid hydration errors
  if (typeof window === 'undefined') {
    // Server-side: always use environment variable or default
    const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
    return (['lan', 'tailscale', 'local'].includes(env) ? env : 'local') as any;
  }
  
  // Client-side: check URL override first, then environment
  try {
    const q = new URL(window.location.href).searchParams.get('profile');
    if (q && ['lan', 'tailscale', 'local'].includes(q)) return q as any;
  } catch {}
  const env = (process.env.NEXT_PUBLIC_NETWORK_PROFILE || 'local').toLowerCase();
  return (['lan', 'tailscale', 'local'].includes(env) ? env : 'local') as any;
};


const Dot: React.FC<{ state: HeaderState }> = ({ state }) => {
  const color =
    state === 'connected'   ? 'bg-emerald-500' :
    state === 'connecting'  ? 'bg-amber-400'  :
    state === 'no_camera'   ? 'bg-sky-500'    :
                              'bg-rose-500';
  return <span className={`inline-block w-2 xl4:w-2.5 h-2 xl4:h-2.5 rounded-full ${color}`} aria-hidden />;
};

type PillProps = {
  label: string;
  state: HeaderState;
  latency?: number | null;
  onClick?: () => void;
  isInteractive?: boolean;
  ariaExpanded?: boolean;
  ariaControls?: string;
  titleOverride?: string;
};

const Pill: React.FC<PillProps> = ({
  label, state, latency, onClick, isInteractive, ariaExpanded, ariaControls, titleOverride,
}) => {
  const glyph =
    state === 'connected'  ? '✓' :
    state === 'connecting' ? '…' :
    state === 'no_camera'  ? '○' : '×';
  const glyphClass =
    state === 'connected'  ? 'text-emerald-400' :
    state === 'connecting' ? 'text-amber-300'  :
    state === 'no_camera'  ? 'text-sky-300'    : 'text-rose-400';

  const base =
    'flex items-center gap-1.5 text-[11px] xl4:text-sm px-2 xl4:px-3 py-1 xl4:py-1.5 rounded-md bg-black/30 border border-white/10';
  const interactive = isInteractive
    ? ' cursor-pointer hover:bg-black/40 focus:outline-none focus:ring-2 focus:ring-white/20'
    : '';

  const title = titleOverride ?? `${label}: ${state}${latency != null ? ` • ${latency} ms` : ''}`;

  return isInteractive ? (
    <button
      type="button"
      className={base + interactive}
      onClick={onClick}
      title={title}
      aria-expanded={ariaExpanded}
      aria-controls={ariaControls}
    >
      <Dot state={state} />
      <span className="text-white/90 truncate max-w-[18ch]">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>{glyph}</span>
      {latency != null && <span className="text-white/50 ml-0.5">{latency} ms</span>}
    </button>
  ) : (
    <div className={base} title={title}>
      <Dot state={state} />
      <span className="text-white/90 truncate max-w-[18ch]">{label}</span>
      <span className={`ml-0.5 ${glyphClass}`} aria-hidden>{glyph}</span>
      {latency != null && <span className="text-white/50 ml-0.5">{latency} ms</span>}
    </div>
  );
};

/* ============================
   Net summary (MVP poller)
   ============================ */
type LinkType = 'wifi' | 'pan' | 'eth' | 'tailscale' | 'none' | 'unknown';
type NetSummary = {
  linkType: LinkType;
  online?: boolean;
  ssid?: string;
  panDevice?: string;
  ifname?: string;
  ipv4?: string;
  ipv6?: string;
  rssi?: number;
  gateway?: string;
};

function useNetSummary(intervalMs = 8000) {
  const [summary, setSummary] = useState<NetSummary | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError]     = useState<string | null>(null);

  const fetchOnce = useCallback(async () => {
    // stale-request guard + timeout
    const ac = new AbortController();
    const t  = setTimeout(() => ac.abort(), 3000);
    try {
      setError(null);
      const res = await fetch('/api/net/summary', { method: 'GET', signal: ac.signal, cache: 'no-store' });
      const ok = res.ok;
      let data: any = null;
      try { data = await res.json(); } catch { data = null; }
      if (!ok) throw new Error(`${res.status} ${res.statusText}`);
      setSummary(data ?? { linkType: 'unknown', online: false });
    } catch (e: any) {
      // Keep last good summary; only inject a minimal one if we have none
      setError(e?.message ?? 'fetch failed');
      setSummary(prev => prev ?? { linkType: 'unknown', online: false });
    } finally {
      clearTimeout(t);
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    let timer: ReturnType<typeof setInterval> | null = null;
    let mounted = true;
    (async () => { if (mounted) await fetchOnce(); })();
    timer = setInterval(() => { void fetchOnce(); }, intervalMs);
    return () => { mounted = false; if (timer) clearInterval(timer); };
  }, [fetchOnce, intervalMs]);

  const state: HeaderState =
    loading ? 'connecting'
    : summary?.online ? 'connected'
    : 'disconnected';

  const label =
    summary?.linkType === 'wifi'      ? `Wi-Fi: ${summary?.ssid ?? '…'}` :
    summary?.linkType === 'pan'       ? `PAN: ${summary?.panDevice ?? 'iPhone'}` :
    summary?.linkType === 'eth'       ? 'Ethernet' :
    summary?.linkType === 'tailscale' ? 'Tailscale' :
    summary?.linkType === 'none'      ? 'Offline' :
                                        'Network';

  const title = [
    summary?.ifname ? `if=${summary.ifname}` : '',
    summary?.ipv4   ? `ipv4=${summary.ipv4}` : '',
    summary?.gateway? `gw=${summary.gateway}`: '',
    summary?.rssi!=null && summary?.linkType==='wifi' ? `rssi=${summary.rssi}dBm` : ''
  ].filter(Boolean).join(' • ');

  return { summary, state, label, title, loading, error, refresh: fetchOnce };
}

/* ============================
   Latency Metrics Hook
   ============================ */
interface LatencyMetrics {
  piOnly?: {
    total_processing_ms?: number;
    encode_duration_ms?: number;
  };
  hybrid?: {
    round_trip_ms?: {
      avg: number;
    };
    inference_ms?: {
      avg: number;
    };
  };
}

// Latency polling: base interval 30s (was 1s = 120 req/min hitting dead endpoints).
// Circuit breaker: after 3 consecutive failures, back off to 60s.
const LATENCY_BASE_INTERVAL   = 30_000;
const LATENCY_BACKOFF_INTERVAL = 60_000;
const LATENCY_FAILURE_THRESHOLD = 3;

function useLatencyMetrics(intervalMs = LATENCY_BASE_INTERVAL) {
  const [metrics, setMetrics] = useState<LatencyMetrics>({});
  const [loading, setLoading] = useState(true);
  const failureCountRef = useRef(0);
  const activeIntervalRef = useRef(intervalMs);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const fetchLatency = useCallback(async () => {
    const ac = new AbortController();
    const t = setTimeout(() => ac.abort(), 2000);
    let fetchFailed = false;
    try {
      const piRes = await fetch('/api/video-proxy/latency', {
        method: 'GET',
        signal: ac.signal,
        cache: 'no-store',
      }).catch(() => null);

      let piData: any = null;
      if (piRes?.ok) {
        try { piData = await piRes.json(); } catch {}
      }

      const hybridRes = await fetch('/api/video-proxy/latency/hybrid', {
        method: 'GET',
        signal: ac.signal,
        cache: 'no-store',
      }).catch(() => null);

      let hybridData: any = null;
      if (hybridRes?.ok) {
        try { hybridData = await hybridRes.json(); } catch {}
      }

      const anyOk = piRes?.ok || hybridRes?.ok;
      if (anyOk) {
        failureCountRef.current = 0;
        setMetrics({
          piOnly: piData?.latencies_ms || undefined,
          hybrid: hybridData?.ok ? {
            round_trip_ms: hybridData.round_trip_ms,
            inference_ms: hybridData.inference_ms,
          } : undefined,
        });
      } else {
        fetchFailed = true;
      }
    } catch {
      fetchFailed = true;
    } finally {
      clearTimeout(t);
      setLoading(false);
    }

    // Circuit breaker: escalate interval on repeated failures
    if (fetchFailed) {
      failureCountRef.current += 1;
      if (failureCountRef.current >= LATENCY_FAILURE_THRESHOLD &&
          activeIntervalRef.current !== LATENCY_BACKOFF_INTERVAL) {
        activeIntervalRef.current = LATENCY_BACKOFF_INTERVAL;
        if (timerRef.current) clearInterval(timerRef.current);
        timerRef.current = setInterval(() => { void fetchLatency(); }, LATENCY_BACKOFF_INTERVAL);
      }
    } else if (activeIntervalRef.current !== intervalMs) {
      // Recover: reset to base interval
      activeIntervalRef.current = intervalMs;
      if (timerRef.current) clearInterval(timerRef.current);
      timerRef.current = setInterval(() => { void fetchLatency(); }, intervalMs);
    }
  }, [intervalMs]);

  useEffect(() => {
    let mounted = true;
    activeIntervalRef.current = intervalMs;
    (async () => { if (mounted) await fetchLatency(); })();
    timerRef.current = setInterval(() => { void fetchLatency(); }, intervalMs);
    return () => {
      mounted = false;
      if (timerRef.current) clearInterval(timerRef.current);
    };
  }, [fetchLatency, intervalMs]);

  return { metrics, loading };
}

/* ============================
   Header
   ============================ */

const Header: React.FC<HeaderProps> = ({ batteryLevel, gamepadConnected = false, gamepadName = '', gamepadPaused = false }) => {
  const router = useRouter();
  const [showServers, setShowServers] = useState(true);
  const { readinessLevel, blockedReason } = useServiceSafety();

  // Resolve endpoints defensively (never throw during render)
  const { MOVE_URL, ULTRA_URL, LINE_URL, LIGHT_URL, VIDEO_URL } = useMemo(() => {
    try {
      return {
        MOVE_URL:  net.ws.movement()  || '',
        ULTRA_URL: net.ws.ultrasonic()|| '',
        LINE_URL:  net.ws.line()      || '',
        LIGHT_URL: net.ws.lighting()  || '',
        VIDEO_URL: net.video()        || '',
      };
    } catch {
      return { MOVE_URL:'', ULTRA_URL:'', LINE_URL:'', LIGHT_URL:'', VIDEO_URL:'' };
    }
  }, []);

  const VIDEO_HEALTH = useMemo(() => toHealthUrl(VIDEO_URL), [VIDEO_URL]);

  // WS with pong → latency
  const move  = useWsStatus(MOVE_URL,  { enabled: !!MOVE_URL,  pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const line  = useWsStatus(LINE_URL,  { enabled: !!LINE_URL,  pingIntervalMs: 5000, pongTimeoutMs: 2500 });
  const light = useWsStatus(LIGHT_URL, { enabled: !!LIGHT_URL, pingIntervalMs: 5000, pongTimeoutMs: 2500 });

  // Streaming WS (any message → alive)
  const ultra = useWsStatus(ULTRA_URL, {
    enabled: !!ULTRA_URL,
    treatAnyMessageAsAlive: true,
    pongTimeoutMs: 4000,
  });

  // HTTP /health (503+placeholder → "no_camera")
  const video = useHttpStatus(VIDEO_HEALTH, {
    enabled: !!VIDEO_HEALTH,
    intervalMs: 5000,
    timeoutMs: 2500,
    treat503AsConnecting: true,
  });

  // Net summary
  const netSummary = useNetSummary(8000);

  // Latency metrics — 30s base interval with circuit breaker (was 1s)
  const latencyMetrics = useLatencyMetrics(LATENCY_BASE_INTERVAL);

  const states = [move.status, ultra.status, line.status, light.status, video.status] as const;
  const upCount = states.filter((s) => s === 'connected').length;
  const allGood = upCount === states.length;

  const batteryClass =
    batteryLevel === null  ? 'bg-gray-600' :
    batteryLevel > 75     ? 'bg-green-500' :
    batteryLevel > 50     ? 'bg-yellow-500' :
    batteryLevel > 20     ? 'bg-blue-500 neon-blue' :
                            'bg-red-500';

  const liveLinkLabel = netSummary.label;
  const profile = getActiveProfile();

  return (
    <div className="bg-gray-800 text-white px-4 xl4:px-10 py-3 xl4:py-5 sticky top-0 z-10 shadow-md">
      <div className="max-w-[1600px] mx-auto flex flex-col gap-2 xl4:gap-4">
      <div className="flex justify-between items-center">
        <div className="text-lg xl4:text-2xl font-bold flex items-center gap-2 xl4:gap-3">
          {/* eslint-disable-next-line @next/next/no-img-element */}
          <img
            src="/image/README/omegatechlogopro-noBackground.png"
            alt="Omega Tech Logo"
            className="h-10 xl4:h-14 w-auto object-contain"
            style={{ filter: 'brightness(1.8) drop-shadow(0 0 6px rgba(139,92,246,0.85))' }}
          />
          Robot Controller
          {/* Profile + MOCK badges (no hostnames = no leaks) */}
          <span className="text-[10px] xl4:text-sm px-1.5 xl4:px-2 py-0.5 xl4:py-1 rounded bg-white/10 border border-white/15">
            {profile.toUpperCase()}
          </span>

          {/* Connection mode picker — click to switch between Live / Sim / Demo */}
          <ConnectionModeBadge />

          {MOCK_WS && (
            <span className="text-[10px] xl4:text-sm px-1.5 xl4:px-2 py-0.5 xl4:py-1 rounded bg-amber-500/20 border border-amber-400/40 text-amber-100">
              MOCK
            </span>
          )}

          {/* Mission Control Link */}
          <div className="text-[10px] xl4:text-sm px-1.5 xl4:px-2.5 py-0.5 xl4:py-1 rounded bg-emerald-500/20 border border-emerald-400/40 text-emerald-100 hover:bg-emerald-500/30 transition-colors">
            <Link href="/mission" title="Mission Control — live map, waypoints, EKF visualization" className="flex items-center gap-1">
              <span>Mission</span>
            </Link>
          </div>

          {/* Network Management Link */}
          <div className="text-[10px] xl4:text-sm px-1.5 xl4:px-2.5 py-0.5 xl4:py-1 rounded bg-blue-500/20 border border-blue-400/40 text-blue-100 hover:bg-blue-500/30 transition-colors">
            <Link href="/network" title="Network Management" className="flex items-center gap-1">
              <Wifi className="w-3 xl4:w-4 h-3 xl4:h-4" />
              <span>Network</span>
            </Link>
          </div>

          {/* ROS Dashboard Link */}
          <div className="text-[10px] xl4:text-sm px-1.5 xl4:px-2.5 py-0.5 xl4:py-1 rounded bg-purple-500/20 border border-purple-400/40 text-purple-100 hover:bg-purple-500/30 transition-colors">
            <Link href="/ros" title="ROS 2 Dashboard (with Debug Tools)">
              ROS
            </Link>
          </div>

          {/* Services Management Link */}
          <div className="text-[10px] xl4:text-sm px-1.5 xl4:px-2.5 py-0.5 xl4:py-1 rounded bg-green-500/20 border border-green-400/40 text-green-100 hover:bg-green-500/30 transition-colors">
            <Link href="/services" title="Service Management">
              Services
            </Link>
          </div>

          {/* Settings Link */}
          <div className="text-[10px] xl4:text-sm px-1.5 xl4:px-2.5 py-0.5 xl4:py-1 rounded bg-yellow-500/20 border border-yellow-400/40 text-yellow-100 hover:bg-yellow-500/30 transition-colors">
            <Link href="/settings" title="Robot Settings" className="flex items-center gap-1">
              <SlidersHorizontal className="w-3 xl4:w-4 h-3 xl4:h-4" />
              <span>Settings</span>
            </Link>
          </div>

          {/* System readiness indicator */}
          <div
            title={
              readinessLevel === 'ready'    ? 'All critical services running' :
              readinessLevel === 'degraded' ? 'Some non-critical services offline' :
              blockedReason ?? 'Critical services offline'
            }
            className={`
              hidden xl4:flex items-center gap-1 text-[10px] xl4:text-sm px-1.5 xl4:px-2.5 py-0.5 xl4:py-1 rounded border font-semibold
              ${readinessLevel === 'ready'    ? 'bg-emerald-500/15 border-emerald-500/40 text-emerald-300' :
                readinessLevel === 'degraded' ? 'bg-amber-500/15 border-amber-500/40 text-amber-300' :
                'bg-rose-500/15 border-rose-500/40 text-rose-300'}
            `}
          >
            <span className={`w-1.5 h-1.5 rounded-full ${
              readinessLevel === 'ready'    ? 'bg-emerald-400' :
              readinessLevel === 'degraded' ? 'bg-amber-400 animate-pulse' :
              'bg-rose-400 animate-pulse'
            }`} />
            {readinessLevel === 'ready' ? 'Ready' : readinessLevel === 'degraded' ? 'Degraded' : 'Critical'}
          </div>
        </div>

        {/* Overall status + battery */}
        <div className="flex items-center space-x-4 xl4:space-x-6 xl4:text-base">
          {/* Pi Connection Status Indicator */}
          <div className="flex items-center text-sm xl4:text-base">
            <span className="opacity-80">Pi:</span>
            {allGood ? (
              // Wrapping the icon because Lucide icons do NOT accept `title`
              <span
                className="inline-flex"
                title="Connected to Pi - All services online"
              >
                <CheckCircle
                  aria-label="Connected to Pi"
                  className="text-green-500 ml-2"
                />
              </span>
            ) : upCount > 0 ? (
              <div className="ml-2 flex items-center gap-1" title={`Partially connected - ${upCount}/${states.length} services online`}>
                <div className="w-2 h-2 rounded-full bg-yellow-500 animate-pulse" />
                <span className="text-yellow-400 text-xs">{upCount}/{states.length}</span>
              </div>
            ) : (
              // Wrapping the icon because Lucide icons do NOT accept `title`
              <span
                className="inline-flex"
                title="Not connected to Pi - Check network settings"
              >
                <XCircle
                  aria-label="Not connected to Pi"
                  className="text-red-500 ml-2"
                />
              </span>
            )}
          </div>
          <div className="flex items-center text-sm xl4:text-base">
            <span className="opacity-80">Status:</span>
            {allGood ? (
              <CheckCircle aria-label="All services reachable" className="text-green-500 ml-2" />
            ) : (
              <XCircle aria-label="Some services down" className="text-red-500 ml-2" />
            )}
            <span className="ml-2 opacity-80">{upCount}/{states.length} online</span>
          </div>

          <div className="flex items-center text-sm xl4:text-base">
            <span className="opacity-80">Battery:</span>
            <div className="ml-2 w-32 xl4:w-48 battery-container">
              <div className={`h-4 xl4:h-5 rounded ${batteryClass}`} style={{ width: `${batteryLevel ?? 0}%` }} />
            </div>
            <span className="ml-2 opacity-80">{batteryLevel != null ? `${batteryLevel}%` : '—'}</span>
          </div>
          
          {/* Capability Status */}
          <div className="flex items-center text-sm ml-4 pl-4 border-l border-white/10">
            <CapabilityInfoModal />
          </div>

          {/* Latency Metrics */}
          <div className="flex items-center text-sm ml-4 pl-4 border-l border-white/10 gap-2">
            <Activity className="w-4 h-4 text-blue-400" />
            {latencyMetrics.metrics.piOnly?.total_processing_ms !== undefined && (
              <div className="flex items-center gap-1" title="Pi-only processing latency">
                <span className="text-white/70 text-xs">Pi:</span>
                <span className="text-blue-300 text-xs font-mono">
                  {latencyMetrics.metrics.piOnly.total_processing_ms.toFixed(1)}ms
                </span>
              </div>
            )}
            {latencyMetrics.metrics.hybrid?.round_trip_ms?.avg !== undefined && (
              <div className="flex items-center gap-1" title="Pi ↔ Orin round-trip latency">
                <span className="text-white/70 text-xs">Hybrid:</span>
                <span className="text-purple-300 text-xs font-mono">
                  {latencyMetrics.metrics.hybrid.round_trip_ms.avg.toFixed(1)}ms
                </span>
                {latencyMetrics.metrics.hybrid.inference_ms?.avg !== undefined && (
                  <span className="text-purple-200/70 text-xs font-mono">
                    ({latencyMetrics.metrics.hybrid.inference_ms.avg.toFixed(1)}ms inf)
                  </span>
                )}
              </div>
            )}
          </div>
        </div>
      </div>

      {/* Servers section with toggle */}
      <div className="flex flex-wrap gap-2 items-center">
        {/* Servers toggle button */}
        <button
          type="button"
          className="flex items-center gap-1.5 text-[11px] px-2 py-1 rounded-md bg-black/30 border border-white/10 cursor-pointer hover:bg-black/40 focus:outline-none focus:ring-2 focus:ring-white/20"
          onClick={() => setShowServers(!showServers)}
          title={`${showServers ? 'Hide' : 'Show'} server status pills`}
        >
          <span className="text-white/90">Servers</span>
          <span className="text-white/50">{showServers ? '▼' : '▶'}</span>
        </button>

        {/* Live Link pill — navigates to Network Management page */}
        <Pill
          label={liveLinkLabel}
          state={netSummary.state}
          titleOverride={netSummary.title ? `${netSummary.title} — click to manage` : 'Click to manage network'}
          isInteractive
          onClick={() => router.push('/network')}
        />

        {/* Server status pills - collapsible */}
        {showServers && (
          <>
            <Pill label="Movement"   state={move.status}   latency={move.latency} />
            <Pill label="Ultrasonic" state={ultra.status} />
            <Pill label="Line"       state={line.status}   latency={line.latency} />
            <Pill label="Lighting"   state={light.status}  latency={light.latency} />
            <Pill label="Video"      state={video.status}  latency={video.latency} />
          </>
        )}

        {/* Xbox Controller pill — always visible */}
        <div
          className="flex items-center gap-1.5 text-[11px] xl4:text-sm px-2 xl4:px-3 py-1 xl4:py-1.5 rounded-md bg-black/30 border border-white/10"
          title={
            gamepadConnected
              ? `Xbox: ${gamepadName || 'Connected'}${gamepadPaused ? ' (paused)' : ' — active'}`
              : 'Xbox: no controller detected'
          }
        >
          <span
            className={`inline-block w-2 xl4:w-2.5 h-2 xl4:h-2.5 rounded-full ${
              gamepadConnected && !gamepadPaused
                ? 'bg-emerald-500'
                : gamepadConnected && gamepadPaused
                ? 'bg-amber-400'
                : 'bg-zinc-600'
            }`}
            aria-hidden
          />
          <span className="text-white/90">Xbox</span>
          <span
            className={
              gamepadConnected && !gamepadPaused
                ? 'text-emerald-400 ml-0.5'
                : gamepadConnected && gamepadPaused
                ? 'text-amber-300 ml-0.5'
                : 'text-zinc-500 ml-0.5'
            }
            aria-hidden
          >
            {gamepadConnected && !gamepadPaused ? '✓' : gamepadConnected && gamepadPaused ? '‖' : '×'}
          </span>
        </div>
      </div>
      </div>
    </div>
  );
};

export default Header;
