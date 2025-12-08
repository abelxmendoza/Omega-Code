/**
 * ServiceAutostartEditor Component
 * 
 * Edits service autostart and restart policies.
 */

import React, { useState, useEffect } from 'react';
import { Save, Loader2, CheckCircle, XCircle } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { useConfigSection } from '@/hooks/useConfigSection';
import { ROBOT_ENABLED } from '@/utils/env';

export function ServiceAutostartEditor() {
  const { section, loading, updateSection } = useConfigSection('services');
  const [saving, setSaving] = useState(false);
  const [autostartServices, setAutostartServices] = useState<string[]>([]);
  const [restartPolicies, setRestartPolicies] = useState<Record<string, string>>({});

  useEffect(() => {
    if (section) {
      setAutostartServices(section.autostart || []);
      setRestartPolicies(section.restart_policies || {});
    }
  }, [section]);

  const toggleAutostart = (serviceName: string) => {
    if (autostartServices.includes(serviceName)) {
      setAutostartServices(autostartServices.filter(s => s !== serviceName));
    } else {
      setAutostartServices([...autostartServices, serviceName]);
    }
  };

  const setRestartPolicy = (serviceName: string, policy: string) => {
    setRestartPolicies({
      ...restartPolicies,
      [serviceName]: policy,
    });
  };

  const handleSave = async () => {
    if (!ROBOT_ENABLED || saving) return;

    setSaving(true);
    try {
      const servicesConfig = {
        autostart: autostartServices,
        restart_policies: restartPolicies,
      };

      const success = await updateSection(servicesConfig);
      if (success) {
        console.log('Services configuration saved');
      }
    } catch (error) {
      console.error('Failed to save services config:', error);
    } finally {
      setSaving(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Service configuration is disabled.
      </div>
    );
  }

  if (loading) {
    return (
      <div className="flex items-center gap-2 text-gray-400">
        <Loader2 className="w-4 h-4 animate-spin" />
        <span>Loading services configuration...</span>
      </div>
    );
  }

  // Get all services from restart policies
  const allServices = Array.from(new Set([
    ...autostartServices,
    ...Object.keys(restartPolicies),
  ]));

  const policyOptions = [
    { value: 'never', label: 'Never', description: 'Do not restart on crash' },
    { value: 'on-failure', label: 'On Failure', description: 'Restart only if service crashes' },
    { value: 'always', label: 'Always', description: 'Always restart on crash' },
  ];

  return (
    <div className="space-y-4">
      <div className="text-sm text-gray-400 mb-4">
        Configure which services start automatically and their restart policies.
      </div>

      <div className="space-y-3">
        {allServices.map((serviceName) => {
          const isAutostart = autostartServices.includes(serviceName);
          const policy = restartPolicies[serviceName] || 'never';

          return (
            <div
              key={serviceName}
              className="p-4 bg-gray-900/50 rounded border border-gray-700"
            >
              <div className="flex items-center justify-between mb-3">
                <div className="flex items-center gap-3">
                  <button
                    onClick={() => toggleAutostart(serviceName)}
                    className={`
                      flex items-center gap-2 px-3 py-1 rounded transition-all
                      ${isAutostart
                        ? 'bg-green-600/20 text-green-400 border border-green-500/50'
                        : 'bg-gray-800 text-gray-400 border border-gray-700'
                      }
                    `}
                  >
                    {isAutostart ? (
                      <>
                        <CheckCircle className="w-4 h-4" />
                        Autostart Enabled
                      </>
                    ) : (
                      <>
                        <XCircle className="w-4 h-4" />
                        Autostart Disabled
                      </>
                    )}
                  </button>
                  <span className="font-medium text-white capitalize">
                    {serviceName.replace(/_/g, ' ')}
                  </span>
                </div>
              </div>

              {isAutostart && (
                <div className="mt-3">
                  <label className="text-sm text-gray-300 mb-2 block">Restart Policy</label>
                  <div className="flex gap-2">
                    {policyOptions.map((option) => (
                      <button
                        key={option.value}
                        onClick={() => setRestartPolicy(serviceName, option.value)}
                        className={`
                          px-3 py-1 rounded text-xs border transition-all
                          ${policy === option.value
                            ? 'border-purple-500 bg-purple-500/10 text-purple-300'
                            : 'border-gray-700 bg-gray-800 text-gray-400 hover:border-gray-600'
                          }
                        `}
                        title={option.description}
                      >
                        {option.label}
                      </button>
                    ))}
                  </div>
                </div>
              )}
            </div>
          );
        })}
      </div>

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
              Save Services Config
            </>
          )}
        </Button>
      </div>
    </div>
  );
}

