'use client';

/**
 * Service Management Page
 *
 * Single polling source (5s, in-flight guarded) owned here.
 * Children receive data as props — no independent polling.
 * Displays last-updated timestamp and exposes manual refresh.
 */

import React, { useState, useEffect } from 'react';
import Head from 'next/head';
import Link from 'next/link';
import { RefreshCw, ArrowLeft, Server, Clock } from 'lucide-react';
import { ROBOT_ENABLED } from '@/utils/env';
import { ServiceTable } from '@/components/services/ServiceTable';
import { ServiceLogs } from '@/components/services/ServiceLogs';
import { useServiceStatus } from '@/hooks/useServiceStatus';

/* ------------------------------------------------------------------ */
/* "X ago" helper — ticks every second                                 */
/* ------------------------------------------------------------------ */

function useTimeAgo(epochMs: number): string {
  const [, tick] = useState(0);

  useEffect(() => {
    if (epochMs === 0) return;
    const id = setInterval(() => tick(n => n + 1), 1000);
    return () => clearInterval(id);
  }, [epochMs]);

  if (epochMs === 0) return 'never';
  const s = Math.floor((Date.now() - epochMs) / 1000);
  if (s < 5)  return 'just now';
  if (s < 60) return `${s}s ago`;
  if (s < 3600) return `${Math.floor(s / 60)}m ago`;
  return `${Math.floor(s / 3600)}h ago`;
}

/* ------------------------------------------------------------------ */
/* Page                                                                */
/* ------------------------------------------------------------------ */

export default function ServicesPage() {
  const { services, loading, lastUpdated, refresh } = useServiceStatus({ interval: 5000 });
  const [selectedService, setSelectedService] = useState<string | null>(null);

  const selectedServiceData = services.find(s => s.name === selectedService);
  const timeAgo = useTimeAgo(lastUpdated);

  const runningCount = services.filter(s => s.status === 'running').length;
  const crashedCount = services.filter(s => s.status === 'crashed').length;

  return (
    <>
      <Head>
        <title>Service Management — Robot Controller</title>
        <meta name="description" content="Manage OmegaOS services" />
      </Head>

      <div className="min-h-screen bg-gray-950 text-white">

        {/* ── Sticky top bar ──────────────────────────────────────── */}
        <div className="sticky top-0 z-10 bg-gray-900 border-b border-white/10 px-4 py-2.5 flex items-center justify-between">
          <div className="flex items-center gap-3">
            <Link
              href="/"
              className="flex items-center gap-1.5 text-xs text-white/50 hover:text-white transition-colors"
            >
              <ArrowLeft size={13} />
              Dashboard
            </Link>
            <span className="text-white/20">|</span>
            <div className="flex items-center gap-1.5">
              <Server size={13} className="text-emerald-400" />
              <span className="text-sm font-bold tracking-wide text-white uppercase">Service Management</span>
            </div>
          </div>

          <div className="flex items-center gap-3">
            {/* Last-updated timestamp */}
            {lastUpdated > 0 && (
              <div className="flex items-center gap-1 text-[11px] text-white/30">
                <Clock size={10} />
                {timeAgo}
              </div>
            )}

            {/* Manual refresh */}
            <button
              onClick={refresh}
              disabled={loading}
              className="flex items-center gap-1.5 px-2.5 py-1 rounded text-xs border border-white/10 text-white/60 hover:bg-white/5 hover:text-white transition-colors disabled:opacity-40"
            >
              <RefreshCw size={12} className={loading ? 'animate-spin' : ''} />
              Refresh
            </button>
          </div>
        </div>

        <div className="max-w-7xl mx-auto px-4 py-6 space-y-4">

          {/* ── Offline guard ───────────────────────────────────── */}
          {!ROBOT_ENABLED ? (
            <div className="bg-gray-900 border border-white/10 rounded-lg p-6 text-center text-white/40 text-sm">
              Robot is offline. Service management is unavailable.
            </div>
          ) : (
            <>
              {/* ── Summary bar ─────────────────────────────────── */}
              {services.length > 0 && (
                <div className="flex items-center gap-4 px-1">
                  <Pill label="Total"   value={services.length}    color="text-white/40" />
                  <Pill label="Running" value={runningCount}        color="text-emerald-400" />
                  {crashedCount > 0 && (
                    <Pill label="Crashed" value={crashedCount}      color="text-rose-400" />
                  )}
                  <div className="ml-auto flex items-center gap-1.5">
                    <span className={`w-1.5 h-1.5 rounded-full ${loading ? 'bg-amber-400 animate-pulse' : 'bg-emerald-400'}`} />
                    <span className="text-[11px] text-white/30">{loading ? 'Fetching…' : 'Live · 5s'}</span>
                  </div>
                </div>
              )}

              {/* ── Service table ───────────────────────────────── */}
              <div className="bg-gray-900 border border-white/10 rounded-lg overflow-hidden">
                <div className="flex items-center gap-2 px-4 py-2.5 bg-gray-800 border-b border-white/10">
                  <Server size={13} className="text-emerald-400" />
                  <span className="text-sm font-bold tracking-wide text-white uppercase">Services</span>
                  {services.length > 0 && (
                    <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 text-white/40 font-medium">
                      {services.length}
                    </span>
                  )}
                </div>

                <ServiceTable
                  services={services}
                  loading={loading}
                  selectedService={selectedService}
                  onServiceSelect={setSelectedService}
                  onActionComplete={refresh}
                />
              </div>

              {/* ── Logs panel ──────────────────────────────────── */}
              {selectedService && selectedServiceData && (
                <div className="bg-gray-900 border border-white/10 rounded-lg overflow-hidden">
                  <ServiceLogs
                    serviceName={selectedService}
                    serviceDisplayName={selectedServiceData.display_name}
                    onClose={() => setSelectedService(null)}
                    autoRefresh
                    refreshInterval={10000}
                  />
                </div>
              )}

              {/* ── Empty state ─────────────────────────────────── */}
              {!loading && services.length === 0 && (
                <div className="bg-gray-900 border border-white/10 rounded-lg p-8 text-center text-white/40 text-sm">
                  No services found. Check that the backend is reachable and the service registry is populated.
                </div>
              )}
            </>
          )}
        </div>
      </div>
    </>
  );
}

/* ------------------------------------------------------------------ */
/* Summary pill                                                        */
/* ------------------------------------------------------------------ */

function Pill({ label, value, color }: { label: string; value: number; color: string }) {
  return (
    <div className="flex items-center gap-1.5 text-xs">
      <span className="text-white/30">{label}</span>
      <span className={`font-bold ${color}`}>{value}</span>
    </div>
  );
}
