/**
 * ProfileSelector Component
 * 
 * Selects robot profile (pi4b, jetson, dev).
 */

import React, { useState, useEffect } from 'react';
import { Loader2, CheckCircle } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { useProfiles } from '@/hooks/useProfiles';
import { useConfigSection } from '@/hooks/useConfigSection';
import { ROBOT_ENABLED } from '@/utils/env';

interface ProfileSelectorProps {
  currentProfile?: string;
  onProfileChange?: (profile: string) => void;
}

export function ProfileSelector({ currentProfile, onProfileChange }: ProfileSelectorProps) {
  const { profiles, activeProfile, loading: profilesLoading } = useProfiles();
  const { section: robotSection, loading: configLoading, updateSection } = useConfigSection('robot');
  const [selectedProfile, setSelectedProfile] = useState<string>(currentProfile || activeProfile || 'pi4b');
  const [saving, setSaving] = useState(false);

  useEffect(() => {
    if (robotSection?.profile) {
      setSelectedProfile(robotSection.profile);
    } else if (activeProfile) {
      setSelectedProfile(activeProfile);
    }
  }, [robotSection, activeProfile]);

  const handleProfileChange = async (profile: string) => {
    if (!ROBOT_ENABLED || saving) return;

    setSaving(true);
    try {
      const success = await updateSection({ profile });
      if (success) {
        setSelectedProfile(profile);
        onProfileChange?.(profile);
      }
    } catch (error) {
      console.error('Failed to update profile:', error);
    } finally {
      setSaving(false);
    }
  };

  if (!ROBOT_ENABLED) {
    return (
      <div className="text-gray-400 text-sm">
        Robot is offline. Profile selection is disabled.
      </div>
    );
  }

  if (profilesLoading || configLoading) {
    return (
      <div className="flex items-center gap-2 text-gray-400">
        <Loader2 className="w-4 h-4 animate-spin" />
        <span>Loading profiles...</span>
      </div>
    );
  }

  const profileOptions = ['pi4b', 'jetson', 'dev'] as const;

  return (
    <div className="space-y-4">
      <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
        {profileOptions.map((profile) => {
          const profileData = profiles[profile];
          const isSelected = selectedProfile === profile;
          const isActive = activeProfile === profile;

          return (
            <button
              key={profile}
              onClick={() => handleProfileChange(profile)}
              disabled={saving}
              className={`
                p-4 rounded-lg border-2 transition-all text-left
                ${isSelected
                  ? 'border-purple-500 bg-purple-500/10'
                  : 'border-gray-700 bg-gray-800 hover:border-gray-600'
                }
                ${saving ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'}
              `}
            >
              <div className="flex items-center justify-between mb-2">
                <span className="font-semibold text-white capitalize">{profile}</span>
                {isSelected && (
                  <CheckCircle className="w-5 h-5 text-purple-400" />
                )}
                {isActive && !isSelected && (
                  <span className="text-xs text-gray-400">(Active)</span>
                )}
              </div>
              {profileData && (
                <div className="text-xs text-gray-400 space-y-1">
                  <div>{profileData.description}</div>
                  <div className="mt-2 pt-2 border-t border-gray-700">
                    <div className="grid grid-cols-2 gap-1">
                      <div>
                        <span className="text-gray-500">CPU:</span>{' '}
                        <span className="text-white">{profileData.hardware.cpu_cores} cores</span>
                      </div>
                      <div>
                        <span className="text-gray-500">RAM:</span>{' '}
                        <span className="text-white">{profileData.hardware.ram_gb}GB</span>
                      </div>
                      <div>
                        <span className="text-gray-500">ML:</span>{' '}
                        <span className={profileData.capabilities.ml_capable ? 'text-green-400' : 'text-gray-500'}>
                          {profileData.capabilities.ml_capable ? 'Yes' : 'No'}
                        </span>
                      </div>
                      <div>
                        <span className="text-gray-500">SLAM:</span>{' '}
                        <span className={profileData.capabilities.slam_capable ? 'text-green-400' : 'text-gray-500'}>
                          {profileData.capabilities.slam_capable ? 'Yes' : 'No'}
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              )}
            </button>
          );
        })}
      </div>

      {selectedProfile && profiles[selectedProfile] && (
        <div className="p-3 bg-gray-900/50 rounded border border-gray-700">
          <div className="text-sm text-gray-300">
            <div className="font-semibold mb-1">Recommended Settings:</div>
            <div className="grid grid-cols-2 gap-2 text-xs">
              <div>
                <span className="text-gray-400">Camera:</span>{' '}
                <span className="text-white">
                  {profiles[selectedProfile].recommended_settings.camera_backend} @{' '}
                  {profiles[selectedProfile].recommended_settings.camera_width}x
                  {profiles[selectedProfile].recommended_settings.camera_height}
                </span>
              </div>
              <div>
                <span className="text-gray-400">Movement:</span>{' '}
                <span className="text-white capitalize">
                  {profiles[selectedProfile].recommended_settings.movement_profile}
                </span>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

