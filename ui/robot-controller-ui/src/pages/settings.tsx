/**
 * Settings Page
 * 
 * Main robot configuration management page.
 */

import React, { useState } from 'react';
import Head from 'next/head';
import Link from 'next/link';
import { SlidersHorizontal, Loader2, ArrowLeft, Wifi } from 'lucide-react';
import { ROBOT_ENABLED } from '@/utils/env';
import { Card, CardContent } from '@/components/ui/card';
import { SettingsSection } from '@/components/settings/SettingsSection';
import { ProfileSelector } from '@/components/settings/ProfileSelector';
import { CameraConfigEditor } from '@/components/settings/CameraConfigEditor';
import { MovementConfigEditor } from '@/components/settings/MovementConfigEditor';
import { LightingConfigEditor } from '@/components/settings/LightingConfigEditor';
import { ServiceAutostartEditor } from '@/components/settings/ServiceAutostartEditor';
import { HardwareMapViewer } from '@/components/settings/HardwareMapViewer';
import { ConfigImportExport } from '@/components/settings/ConfigImportExport';
import { ApplyRestartServices } from '@/components/settings/ApplyRestartServices';
import { useConfig } from '@/hooks/useConfig';
import { useConfigSection } from '@/hooks/useConfigSection';

export default function SettingsPage() {
  const { config, loading: configLoading, refresh } = useConfig();
  const { section: robotSection, updateSection } = useConfigSection('robot');
  const [robotName, setRobotName] = useState('');
  const [savingName, setSavingName] = useState(false);

  React.useEffect(() => {
    if (robotSection?.name) {
      setRobotName(robotSection.name);
    } else if (config?.robot?.name) {
      setRobotName(config.robot.name);
    }
  }, [robotSection, config]);

  const handleSaveRobotName = async () => {
    if (!ROBOT_ENABLED || savingName || !robotName.trim()) return;

    setSavingName(true);
    try {
      const success = await updateSection({ name: robotName.trim() });
      if (success) {
        await refresh();
      }
    } catch (error) {
      console.error('Failed to save robot name:', error);
    } finally {
      setSavingName(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="min-h-screen bg-gray-900 text-white p-4">
        <Head>
          <title>Settings - Robot Controller</title>
        </Head>
        <Card className="bg-gray-800 border-gray-700">
          <CardContent className="p-6">
            <p className="text-gray-400">Robot is offline. Settings are unavailable.</p>
          </CardContent>
        </Card>
      </div>
    );
  }

  return (
    <>
      <Head>
        <title>Settings - Robot Controller</title>
        <meta name="description" content="Configure Omega-1 robot settings" />
      </Head>

      <div className="min-h-screen bg-gray-900 text-white p-4">
        <div className="max-w-6xl mx-auto space-y-6">
          {/* Header */}
          <div className="flex items-center gap-3">
            <Link
              href="/"
              className="flex items-center gap-1.5 text-xs text-white/50 hover:text-white/80 transition-colors"
            >
              <ArrowLeft className="w-3.5 h-3.5" />
              Back
            </Link>
            <div className="w-px h-4 bg-white/15" />
            <SlidersHorizontal className="w-7 h-7 text-yellow-400" />
            <div>
              <h1 className="text-2xl font-bold text-white">Robot Settings</h1>
              <p className="text-gray-400 mt-0.5 text-sm">Configure Omega-1 robot settings</p>
            </div>
          </div>

          {configLoading ? (
            <div className="flex items-center justify-center py-12">
              <Loader2 className="w-8 h-8 animate-spin text-gray-400" />
              <span className="ml-3 text-gray-400">Loading configuration...</span>
            </div>
          ) : (
            <>
              {/* Robot Settings */}
              <SettingsSection
                title="Robot Settings"
                description="Basic robot identification and profile"
              >
                <div className="space-y-4">
                  <div>
                    <label className="text-sm font-medium text-gray-300 mb-2 block">
                      Robot Name
                    </label>
                    <div className="flex gap-2">
                      <input
                        type="text"
                        value={robotName}
                        onChange={(e) => setRobotName(e.target.value)}
                        onBlur={handleSaveRobotName}
                        onKeyDown={(e) => {
                          if (e.key === 'Enter') {
                            handleSaveRobotName();
                          }
                        }}
                        className="flex-1 px-3 py-2 rounded bg-gray-800 border border-gray-700 text-white focus:outline-none focus:ring-2 focus:ring-purple-500"
                        placeholder="Omega-1"
                      />
                      {savingName && (
                        <Loader2 className="w-5 h-5 animate-spin text-gray-400 self-center" />
                      )}
                    </div>
                  </div>

                  <div>
                    <label className="text-sm font-medium text-gray-300 mb-2 block">
                      Robot Profile
                    </label>
                    <ProfileSelector
                      currentProfile={config?.robot?.profile}
                      onProfileChange={refresh}
                    />
                  </div>

                  {config?.robot?.version && (
                    <div className="text-sm text-gray-400">
                      Software Version: <span className="text-white">{config.robot.version}</span>
                    </div>
                  )}
                </div>
              </SettingsSection>

              {/* Network Settings — moved to dedicated page */}
              <SettingsSection
                title="Network Settings"
                description="Wi-Fi, Access Point, and connection profiles"
              >
                <div className="flex items-center justify-between rounded-lg bg-gray-900 border border-white/10 px-4 py-3">
                  <div className="flex items-center gap-3">
                    <Wifi className="w-4 h-4 text-blue-400 shrink-0" />
                    <p className="text-sm text-white/70">
                      Network configuration has moved to the dedicated Network Management page.
                    </p>
                  </div>
                  <Link
                    href="/network"
                    className="ml-4 shrink-0 px-3 py-1.5 rounded-md bg-blue-600/80 hover:bg-blue-600 text-xs font-semibold text-white transition-colors"
                  >
                    Open Network →
                  </Link>
                </div>
              </SettingsSection>

              {/* Camera Settings */}
              <SettingsSection
                title="Camera Settings"
                description="Camera backend, resolution, and FPS"
              >
                <CameraConfigEditor />
              </SettingsSection>

              {/* Movement Settings */}
              <SettingsSection
                title="Movement Settings"
                description="Default movement profile and speed limits"
              >
                <MovementConfigEditor />
              </SettingsSection>

              {/* Lighting Settings */}
              <SettingsSection
                title="Lighting Settings"
                description="Default LED pattern and brightness"
              >
                <LightingConfigEditor />
              </SettingsSection>

              {/* Service Autostart */}
              <SettingsSection
                title="Service Autostart"
                description="Configure which services start automatically"
              >
                <ServiceAutostartEditor />
              </SettingsSection>

              {/* Hardware Map */}
              <SettingsSection
                title="Hardware Map"
                description="View hardware device mapping (read-only)"
                defaultExpanded={false}
              >
                <HardwareMapViewer />
              </SettingsSection>

              {/* Config Import/Export */}
              <SettingsSection
                title="Configuration Import/Export"
                description="Backup and restore configuration"
                defaultExpanded={false}
              >
                <ConfigImportExport />
              </SettingsSection>

              {/* Apply & Restart */}
              <SettingsSection
                title="Apply Changes"
                description="Validate configuration and restart services"
              >
                <ApplyRestartServices />
              </SettingsSection>
            </>
          )}
        </div>
      </div>
    </>
  );
}

