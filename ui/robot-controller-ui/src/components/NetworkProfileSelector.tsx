/**
 * Network Profile Selector Component
 * 
 * Provides UI for selecting and monitoring network profiles
 * (WiFi, Tailscale, Mobile Hotspot) with automatic fallback.
 */

'use client';

import React, { useState, useEffect } from 'react';
import { 
  unifiedNetworkManager, 
  NetworkProfile, 
  NetworkTestResult,
  addNetworkChangeListener 
} from '@/utils/unifiedNetworkManager';

interface NetworkProfileSelectorProps {
  className?: string;
  onProfileChange?: (profile: NetworkProfile) => void;
}

export default function NetworkProfileSelector({ 
  className = '', 
  onProfileChange 
}: NetworkProfileSelectorProps) {
  const [profiles, setProfiles] = useState<NetworkProfile[]>([]);
  const [currentProfile, setCurrentProfile] = useState<NetworkProfile | null>(null);
  const [testResults, setTestResults] = useState<Map<string, NetworkTestResult>>(new Map());
  const [isTesting, setIsTesting] = useState(false);
  const [showDetails, setShowDetails] = useState(false);
  const [isExpanded, setIsExpanded] = useState(false);

  useEffect(() => {
    // Load initial profiles
    setProfiles(unifiedNetworkManager.getAllProfiles());
    setCurrentProfile(unifiedNetworkManager.getCurrentProfile());
    setTestResults(unifiedNetworkManager.getTestResults());

    // Listen for profile changes
    const unsubscribe = addNetworkChangeListener((profile) => {
      setCurrentProfile(profile);
      setTestResults(unifiedNetworkManager.getTestResults());
      onProfileChange?.(profile);
    });

    // Periodic updates
    const interval = setInterval(() => {
      setProfiles(unifiedNetworkManager.getAllProfiles());
      setTestResults(unifiedNetworkManager.getTestResults());
    }, 5000);

    return () => {
      unsubscribe();
      clearInterval(interval);
    };
  }, [onProfileChange]);

  const handleProfileSelect = async (profileId: string) => {
    setIsTesting(true);
    try {
      const success = await unifiedNetworkManager.selectProfile(profileId);
      if (success) {
        console.log(`[NetworkProfileSelector] Switched to profile: ${profileId}`);
      } else {
        console.warn(`[NetworkProfileSelector] Failed to switch to profile: ${profileId}`);
      }
    } catch (error) {
      console.error(`[NetworkProfileSelector] Error switching profile:`, error);
    } finally {
      setIsTesting(false);
    }
  };

  const handleForceFallback = async () => {
    setIsTesting(true);
    try {
      const newProfile = await unifiedNetworkManager.forceFallback();
      if (newProfile) {
        console.log(`[NetworkProfileSelector] Fallback to: ${newProfile.name}`);
      }
    } catch (error) {
      console.error(`[NetworkProfileSelector] Fallback error:`, error);
    } finally {
      setIsTesting(false);
    }
  };

  const getProfileIcon = (type: string) => {
    switch (type) {
      case 'tailscale': return '🔒';
      case 'wifi': return '📶';
      case 'mobile': return '📱';
      case 'direct': return '🔗';
      default: return '🌐';
    }
  };

  const getStatusColor = (isAvailable: boolean, isCurrent: boolean) => {
    if (isCurrent) return 'text-green-400';
    if (isAvailable) return 'text-blue-400';
    return 'text-gray-500';
  };

  const getStatusText = (profile: NetworkProfile) => {
    if (profile.id === currentProfile?.id) return 'Active';
    if (profile.isAvailable) return 'Available';
    return 'Unavailable';
  };

  const formatLatency = (latency?: number) => {
    if (!latency) return 'N/A';
    return `${Math.round(latency)}ms`;
  };

  const formatBandwidth = (bandwidth?: number) => {
    if (!bandwidth) return 'N/A';
    return `${(bandwidth / 1000).toFixed(0)}KB/s`;
  };

  const formatStability = (stability?: number) => {
    if (!stability) return 'N/A';
    return `${Math.round(stability)}%`;
  };

  return (
    <div className={`bg-gray-900 border border-white/10 rounded-lg p-4 ${className}`}>
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-2">
          <h3 className="text-lg font-semibold text-white">Network Profiles</h3>
          {currentProfile && (
            <div className="flex items-center gap-1 text-sm">
              <span>{getProfileIcon(currentProfile.type)}</span>
              <span className="text-gray-300">{currentProfile.name}</span>
              <div className={`w-2 h-2 rounded-full ${
                currentProfile.isAvailable ? 'bg-green-500' : 'bg-red-500'
              }`} />
            </div>
          )}
        </div>
        <div className="flex gap-2">
          <button
            onClick={handleForceFallback}
            disabled={isTesting}
            className="px-3 py-1 text-xs bg-orange-600 hover:bg-orange-700 disabled:bg-gray-600 text-white rounded transition"
          >
            Fallback
          </button>
          <button
            onClick={() => setIsExpanded(!isExpanded)}
            className="px-3 py-1 text-xs bg-gray-700 hover:bg-gray-600 text-white rounded transition"
          >
            {isExpanded ? 'Hide' : 'Show'} Details
          </button>
        </div>
      </div>

      {/* Current Profile Status */}
      {currentProfile && (
        <div className="mb-4 p-3 bg-gray-800 border border-white/10 rounded">
          <div className="flex items-center justify-between mb-2">
            <div className="flex items-center gap-2">
              <span className="text-sm font-medium text-white">Current Profile</span>
              <span className="text-lg">{getProfileIcon(currentProfile.type)}</span>
            </div>
            <div className={`text-sm ${getStatusColor(currentProfile.isAvailable, true)}`}>
              {getStatusText(currentProfile)}
            </div>
          </div>
          
          <div className="grid grid-cols-3 gap-4 text-xs">
            <div>
              <span className="text-gray-400">Latency:</span>
              <span className="ml-2 text-white">{formatLatency(currentProfile.latency)}</span>
            </div>
            <div>
              <span className="text-gray-400">Bandwidth:</span>
              <span className="ml-2 text-white">{formatBandwidth(currentProfile.bandwidth)}</span>
            </div>
            <div>
              <span className="text-gray-400">Stability:</span>
              <span className="ml-2 text-white">{formatStability(currentProfile.stability)}</span>
            </div>
          </div>

          {isExpanded && (
            <div className="mt-3 pt-3 border-t border-white/10">
              <div className="text-xs text-gray-400 mb-2">Optimization Settings:</div>
              <div className="grid grid-cols-2 gap-2 text-xs">
                <div>
                  <span className="text-gray-400">Timeout:</span>
                  <span className="ml-2 text-white">{currentProfile.optimization.timeout}ms</span>
                </div>
                <div>
                  <span className="text-gray-400">Retries:</span>
                  <span className="ml-2 text-white">{currentProfile.optimization.retries}</span>
                </div>
                <div>
                  <span className="text-gray-400">Compression:</span>
                  <span className="ml-2 text-white">{currentProfile.optimization.compression ? 'On' : 'Off'}</span>
                </div>
                <div>
                  <span className="text-gray-400">Video Quality:</span>
                  <span className="ml-2 text-white capitalize">{currentProfile.optimization.videoQuality}</span>
                </div>
              </div>
            </div>
          )}
        </div>
      )}

      {/* Profile List */}
      <div className="space-y-2">
        {profiles.map((profile) => {
          const testResult = testResults.get(profile.id);
          const isCurrent = profile.id === currentProfile?.id;
          
          return (
            <div
              key={profile.id}
              className={`p-3 border rounded transition ${
                isCurrent 
                  ? 'border-green-500 bg-green-900/20' 
                  : profile.isAvailable 
                    ? 'border-blue-500 bg-blue-900/20' 
                    : 'border-gray-600 bg-gray-800'
              }`}
            >
              <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <span className="text-lg">{getProfileIcon(profile.type)}</span>
                  <div>
                    <div className="text-sm font-medium text-white">{profile.name}</div>
                    <div className="text-xs text-gray-400">
                      {profile.gatewayHost}:{profile.gatewayPort}
                    </div>
                  </div>
                </div>
                
                <div className="flex items-center gap-3">
                  <div className="text-right text-xs">
                    <div className={getStatusColor(profile.isAvailable, isCurrent)}>
                      {getStatusText(profile)}
                    </div>
                    {testResult && (
                      <div className="text-gray-400">
                        {formatLatency(testResult.latency)} • {formatStability(testResult.stability)}
                      </div>
                    )}
                  </div>
                  
                  <button
                    onClick={() => handleProfileSelect(profile.id)}
                    disabled={!profile.isAvailable || isCurrent || isTesting}
                    className={`px-3 py-1 text-xs rounded transition ${
                      isCurrent
                        ? 'bg-green-600 text-white cursor-default'
                        : profile.isAvailable
                          ? 'bg-blue-600 hover:bg-blue-700 text-white'
                          : 'bg-gray-600 text-gray-400 cursor-not-allowed'
                    }`}
                  >
                    {isCurrent ? 'Active' : 'Select'}
                  </button>
                </div>
              </div>
            </div>
          );
        })}
      </div>

      {/* Testing Status */}
      {isTesting && (
        <div className="mt-4 p-3 bg-blue-900/20 border border-blue-500/30 rounded">
          <div className="flex items-center gap-2 text-blue-300">
            <div className="animate-spin w-4 h-4 border-2 border-blue-300 border-t-transparent rounded-full" />
            <span className="text-sm">Testing network profiles...</span>
          </div>
        </div>
      )}

      {/* Instructions */}
      <div className="mt-4 text-xs text-gray-400">
        <div className="mb-2">
          <strong>Automatic Selection:</strong> System automatically chooses the best available profile
        </div>
        <div className="mb-2">
          <strong>Manual Selection:</strong> Click "Select" to manually choose a profile
        </div>
        <div className="mb-2">
          <strong>Fallback:</strong> Click "Fallback" to switch to the next available profile
        </div>
        <div>
          <strong>Priority:</strong> Tailscale → WiFi → Mobile → Direct → Local
        </div>
      </div>
    </div>
  );
}
