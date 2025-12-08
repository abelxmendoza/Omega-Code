/**
 * Omega Network Wizard Component
 * 
 * Provides UI for managing Omega-1 Network Wizard (AP/Client mode switching)
 * Integrates with backend REST API: /api/network/*
 */

'use client';

import React, { useState, useEffect, useCallback } from 'react';
import { Wifi, WifiOff, RefreshCw, CheckCircle, XCircle, AlertCircle, Loader2 } from 'lucide-react';
import { robotFetch } from '@/utils/network';

interface NetworkStatus {
  ok: boolean;
  mode: string;
  services: {
    hostapd: string;
    dnsmasq: string;
    dhcpcd: string;
    wpa_supplicant: string;
  };
  wlan0_ip?: string;
  ap_ssid?: string;
  ap_ip?: string;
  last_updated?: string;
}

interface NetworkInfo {
  ok: boolean;
  mode: string;
  services: Record<string, { status: string; enabled: boolean }>;
  interfaces: {
    wlan0: {
      ip?: string;
      ssid?: string;
      status: string;
    };
  };
  ap_config?: {
    ssid?: string;
    ip?: string;
  };
  last_updated?: string;
}

export default function OmegaNetworkWizard() {
  const [status, setStatus] = useState<NetworkStatus | null>(null);
  const [info, setInfo] = useState<NetworkInfo | null>(null);
  const [loading, setLoading] = useState(false);
  const [switching, setSwitching] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  const fetchStatus = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      // Use unified endpoint: GET /api/network
      const response = await robotFetch('/api/network');
      if (response.offline) {
        setError('Robot backend unavailable');
        setLoading(false);
        return;
      }
      const data = await response.json();
      if (data.ok !== false) {
        // Unified endpoint returns summary directly
        setStatus({
          mode: data.mode || 'unknown',
          wlan0_ip: data.ip,
          ap_ssid: data.ap_config?.ssid,
          ap_ip: data.ap_config?.ip,
          services: Object.fromEntries(
            Object.entries(data.services_running || {}).map(([k, v]: [string, any]) => [
              k,
              typeof v === 'object' ? v.status : v
            ])
          ),
          last_updated: data.last_updated,
        });
      } else {
        setError(data.error || 'Failed to fetch network status');
      }
    } catch (err) {
      setError(`Error: ${err instanceof Error ? err.message : 'Unknown error'}`);
    } finally {
      setLoading(false);
    }
  }, []);

  const fetchInfo = useCallback(async () => {
    try {
      // Unified endpoint provides all info
      const response = await robotFetch('/api/network');
      if (response.offline) {
        return;
      }
      const data = await response.json();
      if (data.ok !== false) {
        setInfo({
          ok: true,
          mode: data.mode,
          services: data.services_running || {},
          interfaces: {
            wlan0: {
              ip: data.ip,
              ssid: data.ssid,
              status: data.interface === 'wlan0' ? 'up' : 'down',
            },
          },
          ap_config: data.ap_config,
          last_updated: data.last_updated,
        });
      }
    } catch (err) {
      console.error('Failed to fetch network info:', err);
    }
  }, []);

  useEffect(() => {
    fetchStatus();
    fetchInfo();
    const interval = setInterval(() => {
      fetchStatus();
      fetchInfo();
    }, 5000); // Refresh every 5 seconds
    return () => clearInterval(interval);
  }, [fetchStatus, fetchInfo]);

  const switchMode = useCallback(async (mode: 'ap' | 'client') => {
    setSwitching(true);
    setError(null);
    setSuccess(null);
    
    try {
      const response = await robotFetch('/api/network/mode', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ mode }),
      });

      if (response.offline) {
        setError('Robot backend unavailable');
        setSwitching(false);
        return;
      }

      const data = await response.json();
      
      if (data.ok) {
        setSuccess(data.message || `Switched to ${mode.toUpperCase()} mode`);
        // Refresh status after a delay
        setTimeout(() => {
          fetchStatus();
          fetchInfo();
        }, 2000);
      } else {
        setError(data.detail || 'Failed to switch network mode');
      }
    } catch (err) {
      setError(`Error: ${err instanceof Error ? err.message : 'Unknown error'}`);
    } finally {
      setSwitching(false);
    }
  }, [fetchStatus, fetchInfo]);

  const validateConfig = useCallback(async () => {
    setLoading(true);
    setError(null);
    try {
      const response = await robotFetch('/api/network/validate', {
        method: 'POST',
      });

      if (response.offline) {
        setError('Robot backend unavailable');
        setLoading(false);
        return;
      }

      const data = await response.json();
      
      if (data.ok && data.valid) {
        setSuccess('Network configuration is valid');
      } else {
        const failedChecks = Object.entries(data.results || {})
          .filter(([_, passed]) => !passed)
          .map(([check, _]) => check)
          .join(', ');
        setError(data.error || `Validation failed: ${failedChecks || 'Unknown error'}`);
      }
    } catch (err) {
      setError(`Error: ${err instanceof Error ? err.message : 'Unknown error'}`);
    } finally {
      setLoading(false);
    }
  }, []);

  const getServiceIcon = (serviceStatus: string) => {
    if (serviceStatus === 'active') {
      return <CheckCircle className="w-4 h-4 text-green-400" />;
    }
    return <XCircle className="w-4 h-4 text-red-400" />;
  };

  const getModeColor = (mode: string) => {
    if (mode === 'ap') return 'text-purple-400';
    if (mode === 'client') return 'text-blue-400';
    return 'text-gray-400';
  };

  return (
    <div className="bg-gray-900 border border-gray-700 rounded-lg p-6 shadow-lg max-w-2xl">
      <div className="flex items-center justify-between mb-6">
        <div className="flex items-center gap-3">
          <Wifi className="w-6 h-6 text-purple-400" />
          <h2 className="text-xl font-bold text-white">Omega Network Wizard</h2>
        </div>
        <button
          onClick={() => { fetchStatus(); fetchInfo(); }}
          disabled={loading}
          className="p-2 rounded bg-gray-800 hover:bg-gray-700 text-white transition-colors disabled:opacity-50"
          title="Refresh status"
        >
          <RefreshCw className={`w-5 h-5 ${loading ? 'animate-spin' : ''}`} />
        </button>
      </div>

      {/* Status Messages */}
      {error && (
        <div className="mb-4 p-3 bg-red-900/30 border border-red-500/50 rounded flex items-center gap-2 text-red-200">
          <AlertCircle className="w-5 h-5" />
          <span>{error}</span>
        </div>
      )}

      {success && (
        <div className="mb-4 p-3 bg-green-900/30 border border-green-500/50 rounded flex items-center gap-2 text-green-200">
          <CheckCircle className="w-5 h-5" />
          <span>{success}</span>
        </div>
      )}

      {/* Current Status */}
      {status && (
        <div className="mb-6 space-y-4">
          <div className="bg-gray-800 rounded-lg p-4">
            <div className="flex items-center justify-between mb-3">
              <h3 className="text-lg font-semibold text-white">Current Status</h3>
              <span className={`px-3 py-1 rounded text-sm font-medium ${getModeColor(status.mode)} bg-gray-700`}>
                {status.mode.toUpperCase()} Mode
              </span>
            </div>

            {/* IP Address */}
            {status.wlan0_ip && (
              <div className="mb-2">
                <span className="text-gray-400 text-sm">IP Address:</span>
                <span className="ml-2 text-white font-mono">{status.wlan0_ip}</span>
              </div>
            )}

            {/* AP Configuration */}
            {status.mode === 'ap' && status.ap_ssid && (
              <div className="mb-2 space-y-1">
                <div>
                  <span className="text-gray-400 text-sm">SSID:</span>
                  <span className="ml-2 text-white font-mono">{status.ap_ssid}</span>
                </div>
                {status.ap_ip && (
                  <div>
                    <span className="text-gray-400 text-sm">AP IP:</span>
                    <span className="ml-2 text-white font-mono">{status.ap_ip}</span>
                  </div>
                )}
                <div className="text-xs text-gray-500 mt-2">
                  Connect to <span className="font-mono text-purple-400">{status.ap_ssid}</span> and SSH to{' '}
                  <span className="font-mono text-purple-400">omega1@{status.ap_ip || '192.168.4.1'}</span>
                </div>
              </div>
            )}

            {/* Service Status */}
            <div className="mt-4 pt-4 border-t border-gray-700">
              <h4 className="text-sm font-semibold text-gray-400 mb-2">Services</h4>
              <div className="grid grid-cols-2 gap-2">
                {Object.entries(status.services).map(([service, serviceStatus]) => (
                  <div key={service} className="flex items-center gap-2 text-sm">
                    {getServiceIcon(serviceStatus)}
                    <span className="text-gray-300 capitalize">{service}</span>
                    <span className={`ml-auto text-xs ${serviceStatus === 'active' ? 'text-green-400' : 'text-red-400'}`}>
                      {serviceStatus}
                    </span>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Mode Switching */}
      <div className="mb-6">
        <h3 className="text-lg font-semibold text-white mb-3">Switch Mode</h3>
        <div className="grid grid-cols-2 gap-3">
          <button
            onClick={() => switchMode('ap')}
            disabled={switching || status?.mode === 'ap'}
            className="p-4 rounded-lg bg-purple-900/30 border border-purple-500/50 hover:bg-purple-900/50 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <div className="flex items-center justify-center gap-2">
              {switching && status?.mode !== 'ap' ? (
                <Loader2 className="w-5 h-5 animate-spin text-purple-400" />
              ) : (
                <Wifi className="w-5 h-5 text-purple-400" />
              )}
              <span className="text-white font-medium">AP Mode</span>
            </div>
            <p className="text-xs text-gray-400 mt-2">
              Create Wi-Fi hotspot (Omega1-AP)
            </p>
          </button>

          <button
            onClick={() => switchMode('client')}
            disabled={switching || status?.mode === 'client'}
            className="p-4 rounded-lg bg-blue-900/30 border border-blue-500/50 hover:bg-blue-900/50 transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <div className="flex items-center justify-center gap-2">
              {switching && status?.mode !== 'client' ? (
                <Loader2 className="w-5 h-5 animate-spin text-blue-400" />
              ) : (
                <WifiOff className="w-5 h-5 text-blue-400" />
              )}
              <span className="text-white font-medium">Client Mode</span>
            </div>
            <p className="text-xs text-gray-400 mt-2">
              Connect to existing Wi-Fi
            </p>
          </button>
        </div>
      </div>

      {/* Additional Info */}
      {info && (
        <div className="mb-6 bg-gray-800 rounded-lg p-4">
          <h3 className="text-sm font-semibold text-gray-400 mb-2">Network Details</h3>
          {info.interfaces.wlan0.ssid && (
            <div className="text-sm text-gray-300 mb-1">
              <span className="text-gray-400">Connected to:</span>{' '}
              <span className="font-mono text-white">{info.interfaces.wlan0.ssid}</span>
            </div>
          )}
          {info.last_updated && (
            <div className="text-xs text-gray-500 mt-2">
              Last updated: {new Date(info.last_updated).toLocaleString()}
            </div>
          )}
        </div>
      )}

      {/* Actions */}
      <div className="flex gap-3">
        <button
          onClick={validateConfig}
          disabled={loading}
          className="flex-1 px-4 py-2 rounded bg-gray-800 hover:bg-gray-700 text-white transition-colors disabled:opacity-50 text-sm"
        >
          {loading ? (
            <span className="flex items-center justify-center gap-2">
              <Loader2 className="w-4 h-4 animate-spin" />
              Validating...
            </span>
          ) : (
            'Validate Configuration'
          )}
        </button>
      </div>

      {/* Help Text */}
      <div className="mt-6 pt-4 border-t border-gray-700">
        <p className="text-xs text-gray-500">
          <strong>AP Mode:</strong> Creates a Wi-Fi hotspot. Connect to <span className="font-mono">Omega1-AP</span> and SSH to{' '}
          <span className="font-mono">omega1@192.168.4.1</span>
        </p>
        <p className="text-xs text-gray-500 mt-2">
          <strong>Client Mode:</strong> Connects to your existing Wi-Fi network. Configure credentials separately.
        </p>
      </div>
    </div>
  );
}

