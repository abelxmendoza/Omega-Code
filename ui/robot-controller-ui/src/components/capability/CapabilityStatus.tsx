import React from 'react';
import { useCapabilityContext } from '@/context/CapabilityContext';

export function CapabilityStatus() {
  const {
    capabilities,
    loading,
    error,
    profileMode,
    isMLCapable,
    isSLAMCapable,
    maxResolution,
    maxFPS,
  } = useCapabilityContext();

  if (loading) {
    return (
      <div className="flex items-center gap-2 text-sm text-gray-500">
        <div className="w-2 h-2 bg-yellow-500 rounded-full animate-pulse" />
        <span>Detecting capabilities...</span>
      </div>
    );
  }

  if (error) {
    return (
      <div className="flex items-center gap-2 text-sm text-red-500">
        <div className="w-2 h-2 bg-red-500 rounded-full" />
        <span>Capability detection failed</span>
      </div>
    );
  }

  if (!capabilities) {
    return null;
  }

  const getProfileColor = () => {
    switch (profileMode) {
      case 'jetson':
        return 'bg-green-500';
      case 'lenovo':
        return 'bg-blue-500';
      case 'mac':
        return 'bg-gray-500';
      default:
        return 'bg-gray-400';
    }
  };

  const getProfileLabel = () => {
    switch (profileMode) {
      case 'jetson':
        return 'Omega Mode';
      case 'lenovo':
        return 'Dev Mode';
      case 'mac':
        return 'Light Mode';
      default:
        return 'Unknown';
    }
  };

  return (
    <div className="flex items-center gap-3 text-sm">
      <div className="flex items-center gap-2">
        <div className={`w-2 h-2 rounded-full ${getProfileColor()}`} />
        <span className="font-medium">{getProfileLabel()}</span>
      </div>
      
      <div className="flex items-center gap-4 text-xs text-gray-600">
        {isMLCapable && (
          <span className="flex items-center gap-1" title="GPU-accelerated ML available">
            <span className="text-green-600">●</span>
            ML/GPU
          </span>
        )}
        {isSLAMCapable && (
          <span className="flex items-center gap-1" title="SLAM navigation available">
            <span className="text-blue-600">●</span>
            SLAM
          </span>
        )}
        <span className="text-gray-500" title={`Max camera: ${maxResolution} @ ${maxFPS}fps`}>
          {maxResolution} @ {maxFPS}fps
        </span>
      </div>
    </div>
  );
}

export function CapabilityBadge({ feature }: { feature: string }) {
  const { capabilities } = useCapabilityContext();
  
  if (!capabilities) return null;
  
  const isAvailable = capabilities[feature as keyof typeof capabilities] as boolean;
  
  if (!isAvailable) return null;
  
  return (
    <span className="inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-green-100 text-green-800">
      {feature.replace(/_/g, ' ').toUpperCase()}
    </span>
  );
}

