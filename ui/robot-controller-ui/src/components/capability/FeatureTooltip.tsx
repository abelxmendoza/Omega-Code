import React, { useState } from 'react';
import { AlertTriangle, Lock } from 'lucide-react';
import { useCapabilityContext } from '@/context/CapabilityContext';

interface FeatureTooltipProps {
  feature: string;
  children: React.ReactNode;
  requiredProfile?: 'jetson' | 'lenovo' | 'mac';
}

export function FeatureTooltip({ feature, children, requiredProfile }: FeatureTooltipProps) {
  const { capabilities, profileMode } = useCapabilityContext();
  const [showTooltip, setShowTooltip] = useState(false);

  if (!capabilities) return <>{children}</>;

  const isAvailable = capabilities[feature as keyof typeof capabilities] as boolean;
  const meetsProfile = !requiredProfile || profileMode === requiredProfile;

  if (isAvailable && meetsProfile) {
    return <>{children}</>;
  }

  const getReason = () => {
    if (!isAvailable) {
      switch (feature) {
        case 'ml_capable':
        case 'yolo':
          return 'Requires Jetson Orin Nano with CUDA';
        case 'slam_capable':
          return 'Requires Dev Mode (Lenovo) or Omega Mode (Jetson)';
        case 'face_recognition':
          return profileMode === 'mac' 
            ? 'Requires Dev Mode or Omega Mode'
            : 'Available but slow on CPU (use Jetson for best performance)';
        default:
          return 'Not available in current mode';
      }
    }
    if (!meetsProfile) {
      return `Requires ${requiredProfile} mode`;
    }
    return 'Not available';
  };

  return (
    <div className="relative inline-block">
      <div className="opacity-50 pointer-events-none">{children}</div>
      <div className="absolute inset-0 flex items-center justify-center">
        <button
          onMouseEnter={() => setShowTooltip(true)}
          onMouseLeave={() => setShowTooltip(false)}
          className="text-yellow-500 hover:text-yellow-400 transition-colors"
          type="button"
        >
          <Lock className="h-5 w-5" />
        </button>
      </div>
      {showTooltip && (
        <div className="absolute bottom-full left-1/2 transform -translate-x-1/2 mb-2 w-64 p-3 bg-neutral-900 border border-yellow-500/50 rounded-lg shadow-lg z-50">
          <div className="flex items-start gap-2">
            <AlertTriangle className="h-4 w-4 text-yellow-500 mt-0.5 flex-shrink-0" />
            <div className="text-xs">
              <div className="font-semibold mb-1 text-white">Feature Unavailable</div>
              <div className="text-neutral-300">{getReason()}</div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

