import React, { ReactNode } from 'react';
import { useCapabilityContext } from '@/context/CapabilityContext';

interface CapabilityGateProps {
  feature: 'ml_capable' | 'slam_capable' | 'tracking' | 'aruco' | 'motion_detection' | 'face_recognition' | 'yolo';
  fallback?: ReactNode;
  children: ReactNode;
  mode?: 'jetson' | 'lenovo' | 'mac' | 'all';
}

/**
 * Component that conditionally renders children based on system capabilities.
 * 
 * @example
 * <CapabilityGate feature="ml_capable">
 *   <YOLODetection />
 * </CapabilityGate>
 * 
 * @example
 * <CapabilityGate feature="slam_capable" mode="jetson" fallback={<p>SLAM requires Jetson</p>}>
 *   <SLAMVisualization />
 * </CapabilityGate>
 */
export function CapabilityGate({ feature, fallback = null, children, mode }: CapabilityGateProps) {
  const { capabilities, profileMode } = useCapabilityContext();

  if (!capabilities) {
    return <>{fallback}</>;
  }

  // Check mode requirement
  if (mode && mode !== 'all') {
    if (profileMode !== mode) {
      return <>{fallback}</>;
    }
  }

  // Check feature availability
  const isAvailable = capabilities[feature] as boolean;
  
  if (!isAvailable) {
    return <>{fallback}</>;
  }

  return <>{children}</>;
}

interface ProfileGateProps {
  mode: 'jetson' | 'lenovo' | 'mac';
  fallback?: ReactNode;
  children: ReactNode;
}

/**
 * Component that renders children only in specific profile mode.
 */
export function ProfileGate({ mode, fallback = null, children }: ProfileGateProps) {
  const { profileMode } = useCapabilityContext();

  if (profileMode !== mode) {
    return <>{fallback}</>;
  }

  return <>{children}</>;
}

