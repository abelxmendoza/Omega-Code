'use client';

/**
 * Network Management — single source of truth for all network features.
 *
 * Tabs:
 *  Live    — real-time status, mode switch, Wi-Fi scan/connect, hotspot, Tailscale
 *  Config  — persistent startup config (AP SSID/password/IP/DHCP + client prefs)
 *  Profiles — UI-side connection profile manager
 *  Test    — connection quality / latency testing
 */

import React, { useState, useEffect, useCallback } from 'react';
import Head from 'next/head';
import Link from 'next/link';
import dynamic from 'next/dynamic';
import {
  Wifi, ArrowLeft, Radio, SlidersHorizontal, FlaskConical,
  Users, RefreshCw, Loader2, Monitor,
} from 'lucide-react';
import { NetworkConfigEditor } from '@/components/settings/NetworkConfigEditor';
import NetworkProfileSelector from '@/components/NetworkProfileSelector';
import MobileConnectionTest from '@/components/MobileConnectionTest';
import { robotFetch } from '@/utils/network';

/* ------------------------------------------------------------------ */
/* Dynamic import — OmegaNetworkWizard is large, skip SSR             */
/* ------------------------------------------------------------------ */

const OmegaNetworkWizard = dynamic(
  () => import('@/components/network/OmegaNetworkWizard'),
  {
    ssr: false,
    loading: () => (
      <div className="flex items-center gap-2 text-white/50 text-sm p-6">
        <Loader2 className="w-4 h-4 animate-spin" /> Loading…
      </div>
    ),
  }
);

/* ------------------------------------------------------------------ */
/* AP Connected Clients panel (new)                                    */
/* ------------------------------------------------------------------ */

interface ApClient {
  mac: string;
  ip?: string;
  hostname?: string;
  signal?: number;
  connected_since?: string;
}

function ApClientsPanel() {
  const [clients, setClients] = useState<ApClient[]>([]);
  const [loading, setLoading] = useState(false);
  const [unavailable, setUnavailable] = useState(false);
  const [lastFetched, setLastFetched] = useState<Date | null>(null);

  const fetchClients = useCallback(async () => {
    setLoading(true);
    try {
      const res = await robotFetch('/api/network/ap/clients');
      if ((res as any).offline) { setUnavailable(true); return; }
      if (!res.ok) { setUnavailable(true); return; }
      const data = await res.json();
      if (data.ok === false) { setUnavailable(true); return; }
      setClients(data.clients ?? []);
      setUnavailable(false);
      setLastFetched(new Date());
    } catch {
      setUnavailable(true);
    } finally {
      setLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchClients();
    const t = setInterval(fetchClients, 10_000);
    return () => clearInterval(t);
  }, [fetchClients]);

  return (
    <div className="bg-gray-800 border border-white/10 rounded-lg overflow-hidden">
      <div className="flex items-center justify-between px-4 py-3 border-b border-white/10">
        <div className="flex items-center gap-2">
          <Users className="w-4 h-4 text-purple-400" />
          <span className="text-sm font-semibold text-white">Connected Clients</span>
          {!unavailable && (
            <span className="text-xs px-1.5 py-0.5 rounded bg-white/10 text-white/60">
              {clients.length}
            </span>
          )}
        </div>
        <button
          onClick={fetchClients}
          disabled={loading}
          className="p-1.5 rounded hover:bg-white/10 text-white/60 hover:text-white transition-colors disabled:opacity-40"
          title="Refresh client list"
        >
          <RefreshCw className={`w-3.5 h-3.5 ${loading ? 'animate-spin' : ''}`} />
        </button>
      </div>

      <div className="p-4">
        {unavailable ? (
          <p className="text-xs text-white/40 italic">
            Client list not available — robot may not support this endpoint, or AP mode is not active.
          </p>
        ) : clients.length === 0 ? (
          <p className="text-xs text-white/40 italic">
            {loading ? 'Fetching…' : 'No clients connected.'}
          </p>
        ) : (
          <div className="space-y-2">
            {clients.map((c) => (
              <div
                key={c.mac}
                className="flex items-center justify-between rounded-lg bg-gray-900/60 border border-white/10 px-3 py-2 text-sm"
              >
                <div className="flex flex-col gap-0.5">
                  <span className="text-white font-mono text-xs">{c.mac}</span>
                  {c.hostname && (
                    <span className="text-white/70 text-xs">{c.hostname}</span>
                  )}
                </div>
                <div className="flex flex-col items-end gap-0.5 text-right">
                  {c.ip && (
                    <span className="text-white/70 font-mono text-xs">{c.ip}</span>
                  )}
                  {c.signal !== undefined && (
                    <span className="text-white/40 text-[10px]">{c.signal} dBm</span>
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
        {lastFetched && (
          <p className="text-[10px] text-white/25 mt-3">
            Updated {lastFetched.toLocaleTimeString()}
          </p>
        )}
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ */
/* Tabs                                                                */
/* ------------------------------------------------------------------ */

const TABS = [
  { id: 'live',     label: 'Live',     icon: Monitor },
  { id: 'ap',       label: 'Access Point', icon: Radio },
  { id: 'config',   label: 'Config',   icon: SlidersHorizontal },
  { id: 'profiles', label: 'Profiles', icon: Wifi },
  { id: 'test',     label: 'Test',     icon: FlaskConical },
] as const;

type TabId = typeof TABS[number]['id'];

/* ------------------------------------------------------------------ */
/* Page                                                                */
/* ------------------------------------------------------------------ */

export default function NetworkPage() {
  const [tab, setTab] = useState<TabId>('live');

  return (
    <>
      <Head>
        <title>Network Management — Robot Controller</title>
        <meta name="description" content="Manage all network connections for Omega-1" />
      </Head>

      <div className="min-h-screen bg-gray-950 text-white">
        {/* Sticky top bar */}
        <div className="sticky top-0 z-10 bg-gray-900 border-b border-white/10 px-4 py-2.5 flex items-center justify-between">
          <div className="flex items-center gap-3">
            <Link
              href="/"
              className="flex items-center gap-1.5 text-xs text-white/50 hover:text-white transition-colors"
            >
              <ArrowLeft className="w-3.5 h-3.5" />
              Dashboard
            </Link>
            <span className="text-white/20">|</span>
            <Wifi className="w-4 h-4 text-blue-400" />
            <span className="text-sm font-bold tracking-wide text-white uppercase">Network Management</span>
          </div>
        </div>

        <div className="max-w-4xl mx-auto px-4 py-6 space-y-5">

          {/* ── Tab bar ──────────────────────────────────────── */}
          <div className="flex gap-1 bg-gray-800 border border-white/10 rounded-lg p-1 overflow-x-auto">
            {TABS.map(({ id, label, icon: Icon }) => (
              <button
                key={id}
                onClick={() => setTab(id)}
                className={[
                  'flex items-center gap-1.5 px-3 py-2 rounded-md text-sm font-medium whitespace-nowrap transition-colors flex-1 justify-center',
                  tab === id
                    ? 'bg-blue-600/80 text-white shadow-sm'
                    : 'text-white/60 hover:text-white hover:bg-white/10',
                ].join(' ')}
              >
                <Icon className="w-3.5 h-3.5" />
                {label}
              </button>
            ))}
          </div>

          {/* ── Tab content ──────────────────────────────────── */}

          {tab === 'live' && (
            <div className="space-y-4">
              <OmegaNetworkWizard />
            </div>
          )}

          {tab === 'ap' && (
            <div className="space-y-4">
              <div className="text-xs text-white/50 px-1">
                Manage the robot&apos;s built-in Wi-Fi Access Point — configure its SSID, password, and IP range, and see which devices are connected.
              </div>
              {/* AP config (persistent settings saved to robot) */}
              <div className="bg-gray-800 border border-white/10 rounded-lg p-5">
                <div className="flex items-center gap-2 mb-4">
                  <Radio className="w-4 h-4 text-purple-400" />
                  <h2 className="text-sm font-semibold text-white">Access Point Configuration</h2>
                </div>
                <NetworkConfigEditor />
              </div>
              {/* Connected clients */}
              <ApClientsPanel />
            </div>
          )}

          {tab === 'config' && (
            <div className="space-y-4">
              <div className="text-xs text-white/50 px-1">
                Persistent startup configuration — these settings take effect on next reboot or when you apply them from the robot backend.
              </div>
              <div className="bg-gray-800 border border-white/10 rounded-lg p-5">
                <div className="flex items-center gap-2 mb-4">
                  <SlidersHorizontal className="w-4 h-4 text-yellow-400" />
                  <h2 className="text-sm font-semibold text-white">Default Mode &amp; Client Settings</h2>
                </div>
                <NetworkConfigEditor />
              </div>
            </div>
          )}

          {tab === 'profiles' && (
            <div className="space-y-4">
              <div className="text-xs text-white/50 px-1">
                UI-side connection profiles control which IP/hostname the browser uses to reach the robot. Switching profiles does not change the robot&apos;s network — it changes how <em>this browser</em> connects.
              </div>
              <NetworkProfileSelector />
            </div>
          )}

          {tab === 'test' && (
            <div className="space-y-4">
              <div className="text-xs text-white/50 px-1">
                Test latency, bandwidth, and WebSocket connectivity to all robot services.
              </div>
              <MobileConnectionTest />
            </div>
          )}

        </div>
      </div>
    </>
  );
}
