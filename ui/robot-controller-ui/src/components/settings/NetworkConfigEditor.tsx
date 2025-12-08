/**
 * NetworkConfigEditor Component
 * 
 * Edits network configuration (AP/Client mode).
 */

import React, { useState, useEffect } from 'react';
import { Save, Loader2 } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { useConfigSection } from '@/hooks/useConfigSection';
import { validateNetworkMode, validateIPAddress, validateSSID, validateWiFiPassword } from '@/utils/validators';
import { ROBOT_ENABLED } from '@/utils/env';

export function NetworkConfigEditor() {
  const { section, loading, updateSection } = useConfigSection('network');
  const [saving, setSaving] = useState(false);
  const [errors, setErrors] = useState<Record<string, string>>({});

  const [defaultMode, setDefaultMode] = useState('ap');
  const [apSSID, setApSSID] = useState('');
  const [apPassword, setApPassword] = useState('');
  const [apIP, setApIP] = useState('');
  const [apDHCPStart, setApDHCPStart] = useState('');
  const [apDHCPEnd, setApDHCPEnd] = useState('');
  const [clientAutoConnect, setClientAutoConnect] = useState(true);
  const [clientPreferredSSID, setClientPreferredSSID] = useState('');
  const [clientStaticIP, setClientStaticIP] = useState<string | null>(null);

  useEffect(() => {
    if (section) {
      setDefaultMode(section.default_mode || 'ap');
      if (section.ap) {
        setApSSID(section.ap.ssid || '');
        setApPassword(section.ap.password || '');
        setApIP(section.ap.ip || '');
        setApDHCPStart(section.ap.dhcp_start || '');
        setApDHCPEnd(section.ap.dhcp_end || '');
      }
      if (section.client) {
        setClientAutoConnect(section.client.auto_connect ?? true);
        setClientPreferredSSID(section.client.preferred_ssid || '');
        setClientStaticIP(section.client.static_ip || null);
      }
    }
  }, [section]);

  const validate = (): boolean => {
    const newErrors: Record<string, string> = {};

    // Validate default mode
    const modeValidation = validateNetworkMode(defaultMode);
    if (!modeValidation.valid) {
      newErrors.defaultMode = modeValidation.errors[0];
    }

    // Validate AP settings
    if (defaultMode === 'ap') {
      const ssidValidation = validateSSID(apSSID);
      if (!ssidValidation.valid) {
        newErrors.apSSID = ssidValidation.errors[0];
      }

      const passwordValidation = validateWiFiPassword(apPassword);
      if (!passwordValidation.valid) {
        newErrors.apPassword = passwordValidation.errors[0];
      }

      const ipValidation = validateIPAddress(apIP);
      if (!ipValidation.valid) {
        newErrors.apIP = ipValidation.errors[0];
      }

      const dhcpStartValidation = validateIPAddress(apDHCPStart);
      if (!dhcpStartValidation.valid) {
        newErrors.apDHCPStart = dhcpStartValidation.errors[0];
      }

      const dhcpEndValidation = validateIPAddress(apDHCPEnd);
      if (!dhcpEndValidation.valid) {
        newErrors.apDHCPEnd = dhcpEndValidation.errors[0];
      }
    }

    // Validate client static IP if set
    if (clientStaticIP) {
      const ipValidation = validateIPAddress(clientStaticIP);
      if (!ipValidation.valid) {
        newErrors.clientStaticIP = ipValidation.errors[0];
      }
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSave = async () => {
    if (!ROBOT_ENABLED || saving || !validate()) return;

    setSaving(true);
    try {
      const networkConfig: any = {
        default_mode: defaultMode,
        ap: {
          ssid: apSSID,
          password: apPassword,
          ip: apIP,
          dhcp_start: apDHCPStart,
          dhcp_end: apDHCPEnd,
        },
        client: {
          auto_connect: clientAutoConnect,
          preferred_ssid: clientPreferredSSID || undefined,
          static_ip: clientStaticIP || null,
        },
      };

      const success = await updateSection(networkConfig);
      if (success) {
        // Show success toast (would integrate with toast system)
        console.log('Network configuration saved');
      }
    } catch (error) {
      console.error('Failed to save network config:', error);
    } finally {
      setSaving(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Network configuration is disabled.
      </div>
    );
  }

  if (loading) {
    return (
      <div className="flex items-center gap-2 text-gray-400">
        <Loader2 className="w-4 h-4 animate-spin" />
        <span>Loading network configuration...</span>
      </div>
    );
  }

  return (
    <div className="space-y-6">
      {/* Default Mode */}
      <div>
        <Label className="text-white mb-2 block">Default Network Mode</Label>
        <div className="flex gap-3">
          <button
            onClick={() => setDefaultMode('ap')}
            className={`
              px-4 py-2 rounded border-2 transition-all
              ${defaultMode === 'ap'
                ? 'border-purple-500 bg-purple-500/10 text-white'
                : 'border-gray-700 bg-gray-800 text-gray-300 hover:border-gray-600'
              }
            `}
          >
            Access Point (AP)
          </button>
          <button
            onClick={() => setDefaultMode('client')}
            className={`
              px-4 py-2 rounded border-2 transition-all
              ${defaultMode === 'client'
                ? 'border-purple-500 bg-purple-500/10 text-white'
                : 'border-gray-700 bg-gray-800 text-gray-300 hover:border-gray-600'
              }
            `}
          >
            Wi-Fi Client
          </button>
        </div>
        {errors.defaultMode && (
          <p className="text-red-400 text-xs mt-1">{errors.defaultMode}</p>
        )}
      </div>

      {/* AP Mode Settings */}
      {defaultMode === 'ap' && (
        <div className="space-y-4 p-4 bg-gray-900/50 rounded border border-gray-700">
          <h4 className="text-sm font-semibold text-white">Access Point Settings</h4>
          
          <div>
            <Label htmlFor="ap-ssid" className="text-gray-300">SSID</Label>
            <Input
              id="ap-ssid"
              value={apSSID}
              onChange={(e) => setApSSID(e.target.value)}
              className="bg-gray-800 border-gray-700 text-white"
              placeholder="Omega1-AP"
            />
            {errors.apSSID && (
              <p className="text-red-400 text-xs mt-1">{errors.apSSID}</p>
            )}
          </div>

          <div>
            <Label htmlFor="ap-password" className="text-gray-300">Password</Label>
            <Input
              id="ap-password"
              type="password"
              value={apPassword}
              onChange={(e) => setApPassword(e.target.value)}
              className="bg-gray-800 border-gray-700 text-white"
              placeholder="omegawifi123"
            />
            {errors.apPassword && (
              <p className="text-red-400 text-xs mt-1">{errors.apPassword}</p>
            )}
          </div>

          <div>
            <Label htmlFor="ap-ip" className="text-gray-300">AP IP Address</Label>
            <Input
              id="ap-ip"
              value={apIP}
              onChange={(e) => setApIP(e.target.value)}
              className="bg-gray-800 border-gray-700 text-white"
              placeholder="192.168.4.1"
            />
            {errors.apIP && (
              <p className="text-red-400 text-xs mt-1">{errors.apIP}</p>
            )}
          </div>

          <div className="grid grid-cols-2 gap-4">
            <div>
              <Label htmlFor="ap-dhcp-start" className="text-gray-300">DHCP Start</Label>
              <Input
                id="ap-dhcp-start"
                value={apDHCPStart}
                onChange={(e) => setApDHCPStart(e.target.value)}
                className="bg-gray-800 border-gray-700 text-white"
                placeholder="192.168.4.2"
              />
              {errors.apDHCPStart && (
                <p className="text-red-400 text-xs mt-1">{errors.apDHCPStart}</p>
              )}
            </div>

            <div>
              <Label htmlFor="ap-dhcp-end" className="text-gray-300">DHCP End</Label>
              <Input
                id="ap-dhcp-end"
                value={apDHCPEnd}
                onChange={(e) => setApDHCPEnd(e.target.value)}
                className="bg-gray-800 border-gray-700 text-white"
                placeholder="192.168.4.20"
              />
              {errors.apDHCPEnd && (
                <p className="text-red-400 text-xs mt-1">{errors.apDHCPEnd}</p>
              )}
            </div>
          </div>
        </div>
      )}

      {/* Client Mode Settings */}
      {defaultMode === 'client' && (
        <div className="space-y-4 p-4 bg-gray-900/50 rounded border border-gray-700">
          <h4 className="text-sm font-semibold text-white">Client Mode Settings</h4>
          
          <div className="flex items-center gap-2">
            <input
              type="checkbox"
              id="client-auto-connect"
              checked={clientAutoConnect}
              onChange={(e) => setClientAutoConnect(e.target.checked)}
              className="w-4 h-4 rounded border-gray-600 bg-gray-700 text-purple-500"
            />
            <Label htmlFor="client-auto-connect" className="text-gray-300">
              Auto-connect to preferred network
            </Label>
          </div>

          <div>
            <Label htmlFor="client-preferred-ssid" className="text-gray-300">Preferred SSID</Label>
            <Input
              id="client-preferred-ssid"
              value={clientPreferredSSID}
              onChange={(e) => setClientPreferredSSID(e.target.value)}
              className="bg-gray-800 border-gray-700 text-white"
              placeholder="YourWiFiNetwork"
            />
          </div>

          <div>
            <Label htmlFor="client-static-ip" className="text-gray-300">Static IP (optional, leave empty for DHCP)</Label>
            <Input
              id="client-static-ip"
              value={clientStaticIP || ''}
              onChange={(e) => setClientStaticIP(e.target.value || null)}
              className="bg-gray-800 border-gray-700 text-white"
              placeholder="192.168.1.100"
            />
            {errors.clientStaticIP && (
              <p className="text-red-400 text-xs mt-1">{errors.clientStaticIP}</p>
            )}
          </div>
        </div>
      )}

      {/* Save Button */}
      <div className="flex justify-end">
        <Button
          onClick={handleSave}
          disabled={saving}
          className="bg-purple-600 hover:bg-purple-700 text-white"
        >
          {saving ? (
            <>
              <Loader2 className="w-4 h-4 mr-2 animate-spin" />
              Saving...
            </>
          ) : (
            <>
              <Save className="w-4 h-4 mr-2" />
              Save Network Config
            </>
          )}
        </Button>
      </div>
    </div>
  );
}

