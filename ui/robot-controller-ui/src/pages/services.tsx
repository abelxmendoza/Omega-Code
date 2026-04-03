'use client';

/**
 * Service Management Page
 *
 * Manage OmegaOS robot services — start, stop, restart, and tail logs.
 * Single polling source (5s) owned here; ServiceTable receives data as props.
 */

import React, { useState } from 'react';
import Head from 'next/head';
import Link from 'next/link';
import { RefreshCw, ArrowLeft, Server } from 'lucide-react';
import { ROBOT_ENABLED } from '@/utils/env';
import { ServiceTable } from '@/components/services/ServiceTable';
import { ServiceLogs } from '@/components/services/ServiceLogs';
import { useServiceStatus } from '@/hooks/useServiceStatus';

export default function ServicesPage() {
  const { services, loading, refresh } = useServiceStatus({ interval: 5000 });
  const [selectedService, setSelectedService] = useState<string | null>(null);

  const selectedServiceData = services.find(s => s.name === selectedService);

  return (
    <>
      <Head>
        <title>Service Management — Robot Controller</title>
        <meta name="description" content="Manage OmegaOS services" />
      </Head>

      <div className="min-h-screen bg-gray-950 text-white">

        {/* Top bar */}
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

          <button
            onClick={refresh}
            disabled={loading}
            className="flex items-center gap-1.5 px-2.5 py-1 rounded text-xs border border-white/10 text-white/60 hover:bg-white/5 hover:text-white transition-colors disabled:opacity-40"
          >
            <RefreshCw size={12} className={loading ? 'animate-spin' : ''} />
            Refresh
          </button>
        </div>

        <div className="max-w-7xl mx-auto px-4 py-6 space-y-4">

          {/* Offline guard */}
          {!ROBOT_ENABLED ? (
            <div className="bg-gray-900 border border-white/10 rounded-lg p-6 text-center text-white/40 text-sm">
              Robot is offline. Service management is unavailable.
            </div>
          ) : (
            <>
              {/* Service table */}
              <div className="bg-gray-900 border border-white/10 rounded-lg overflow-hidden">
                <div className="flex items-center justify-between px-4 py-2.5 bg-gray-800 border-b border-white/10">
                  <div className="flex items-center gap-2">
                    <Server size={13} className="text-emerald-400" />
                    <span className="text-sm font-bold tracking-wide text-white uppercase">Services</span>
                    {services.length > 0 && (
                      <span className="text-[10px] px-1.5 py-0.5 rounded bg-white/10 text-white/40 font-medium">
                        {services.length}
                      </span>
                    )}
                  </div>
                  <div className="flex items-center gap-1.5">
                    <span className={`w-1.5 h-1.5 rounded-full ${loading ? 'bg-amber-400 animate-pulse' : 'bg-emerald-400'}`} />
                    <span className="text-[11px] text-white/40">{loading ? 'Polling…' : 'Live'}</span>
                  </div>
                </div>

                <ServiceTable
                  services={services}
                  loading={loading}
                  onServiceSelect={setSelectedService}
                  selectedService={selectedService}
                  onActionComplete={refresh}
                />
              </div>

              {/* Logs panel */}
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

              {/* Empty state */}
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
