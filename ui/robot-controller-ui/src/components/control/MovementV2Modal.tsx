/*
# File: /src/components/control/MovementV2Modal.tsx
# Summary:
# Modal component displaying Movement V2 status including:
# - Active profile (smooth/aggressive/precision)
# - Thermal safety state and temperature
# - Ramping visualization (current vs target PWM)
# - Watchdog countdown timer
# - PID status (if available)
*/

import React, { useEffect, useState } from 'react';
import { createPortal } from 'react-dom';
import { MovementV2Data, useCommand } from '@/context/CommandContext';
import { COMMAND } from '@/control_definitions';

interface MovementV2ModalProps {
  isOpen: boolean;
  onClose: () => void;
  movementV2: MovementV2Data | null;
}

const MovementV2Modal: React.FC<MovementV2ModalProps> = ({ isOpen, onClose, movementV2 }) => {
  const { sendCommand } = useCommand();
  const [isChangingProfile, setIsChangingProfile] = useState(false);

  // Close modal on Escape key press
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };
    window.addEventListener('keydown', handleEscape);
    return () => window.removeEventListener('keydown', handleEscape);
  }, [isOpen, onClose]);

  const handleProfileChange = async (profileName: string) => {
    setIsChangingProfile(true);
    try {
      sendCommand(COMMAND.SET_PROFILE, { profile: profileName });
      // Request status update after a short delay to get updated profile
      setTimeout(() => {
        sendCommand(COMMAND.STATUS);
        setIsChangingProfile(false);
      }, 300);
    } catch (error) {
      console.error('Failed to change profile:', error);
      setIsChangingProfile(false);
    }
  };

  if (!isOpen) return null;

  // Handle missing Movement V2 data
  if (!movementV2 || !movementV2.enabled) {
    if (!movementV2) {
      console.error('❌ [UI][V2] Movement V2 data is null/undefined');
    } else if (!movementV2.enabled) {
      console.warn('⚠️ [UI][V2] Movement V2 is disabled');
    }
    const notAvailableContent = (
      <div
        className="fixed inset-0 z-50 flex items-center justify-center bg-black/80 backdrop-blur-md p-4"
        onClick={onClose}
        style={{ fontFamily: "'Rajdhani', 'Exo 2', sans-serif" }}
      >
        <div
          className="bg-[#0A0A0A] rounded-lg max-w-md w-full border-2 border-[#C400FF]/70 shadow-2xl text-[#E0E0E0]"
          onClick={(e) => e.stopPropagation()}
          style={{
            boxShadow: '0 0 40px rgba(196, 0, 255, 0.3), 0 8px 32px rgba(0, 0, 0, 0.6), inset 0 0 40px rgba(196, 0, 255, 0.1)',
          }}
        >
          <div className="flex items-center justify-between p-6 border-b border-[#C400FF]/30">
            <h2 className="text-xl font-bold text-white" style={{ fontFamily: "'Orbitron', sans-serif" }}>Movement V2 Status</h2>
            <button
              onClick={onClose}
              className="text-[#E0E0E0] hover:text-white transition-colors p-2 rounded hover:bg-[#1A1A1A]"
              aria-label="Close modal"
            >
              <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>
          <div className="text-center py-8 px-6">
            <p className="text-[#B0B0B0]">Movement V2 Not Available</p>
            <p className="text-sm text-[#808080] mt-2">
              {(movementV2 as any)?.available === false
                ? 'Movement V2 modules are not available on the backend'
                : 'Movement V2 is not enabled'}
            </p>
          </div>
          {/* Exit Button */}
          <div className="flex justify-end p-6 pt-4 border-t border-[#C400FF]/30">
            <button
              onClick={onClose}
              className="px-6 py-2 bg-gradient-to-r from-[#4B0082] to-[#1A1A1A] hover:from-[#6B0082] hover:to-[#2A2A2A] text-[#E0E0E0] rounded-lg transition-all text-sm font-medium border border-[#C400FF]/40 hover:border-[#C400FF] hover:shadow-[0_0_15px_rgba(196,0,255,0.3)]"
            >
              Exit
            </button>
          </div>
        </div>
      </div>
    );
    
    return typeof document !== 'undefined'
      ? createPortal(notAvailableContent, document.body)
      : null;
  }

  // Get thermal state color
  const getThermalColor = (state?: string) => {
    switch (state) {
      case 'ok':
        return 'text-green-400';
      case 'warning':
        return 'text-yellow-400';
      case 'throttle':
        return 'text-orange-400';
      case 'kill':
        return 'text-red-400';
      default:
        return 'text-gray-400';
    }
  };

  // Calculate ramping percentage
  const getRampingPercentage = () => {
    if (!movementV2.ramping) return 0;
    const { current_pwm, target_pwm } = movementV2.ramping;
    if (target_pwm === 0) return 0;
    const percent = Math.abs(current_pwm) / Math.max(Math.abs(target_pwm), 1) * 100;
    return Math.min(100, Math.max(0, percent));
  };

  const rampingPercent = getRampingPercentage();

  const modalContent = (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center bg-black/80 backdrop-blur-md p-4"
      onClick={onClose}
      style={{ fontFamily: "'Rajdhani', 'Exo 2', sans-serif" }}
    >
      <div
        className="bg-[#0A0A0A] rounded-lg max-w-md w-full max-h-[90vh] overflow-y-auto border-2 border-[#C400FF]/70 shadow-2xl text-[#E0E0E0]"
        onClick={(e) => e.stopPropagation()}
        style={{
          boxShadow: '0 0 40px rgba(196, 0, 255, 0.3), 0 8px 32px rgba(0, 0, 0, 0.6), inset 0 0 40px rgba(196, 0, 255, 0.1)',
        }}
      >
        {/* Header */}
        <div className="flex items-center justify-between p-6 border-b border-[#C400FF]/30">
          <h2 className="text-xl font-bold text-white" style={{ fontFamily: "'Orbitron', sans-serif" }}>Movement V2 Status</h2>
          <button
            onClick={onClose}
            className="text-[#E0E0E0] hover:text-white transition-colors p-2 rounded hover:bg-[#1A1A1A]"
            aria-label="Close modal"
          >
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
            </svg>
          </button>
        </div>

        {/* Content */}
        <div className="p-6 space-y-4">
          {/* Profile Section */}
          {movementV2.profile && (
            <div className="bg-[#1A1A1A] border border-[#C400FF]/30 rounded-lg p-4 backdrop-blur-sm" style={{ boxShadow: '0 0 20px rgba(196, 0, 255, 0.1)' }}>
              <div className="flex items-center justify-between mb-3">
                <span className="text-[#B0B0B0] text-sm">Profile:</span>
                <span className="text-[#C400FF] font-medium capitalize">
                  {movementV2.profile.name || 'Unknown'}
                </span>
              </div>
              <div className="flex gap-2">
                {['smooth', 'aggressive', 'precision'].map((profile) => (
                  <button
                    key={profile}
                    onClick={() => handleProfileChange(profile)}
                    disabled={isChangingProfile || movementV2.profile?.name === profile}
                    className={`px-3 py-1.5 text-xs rounded-lg transition-all border ${
                      movementV2.profile?.name === profile
                        ? 'bg-[#C400FF]/20 border-[#C400FF] text-[#C400FF]'
                        : 'bg-[#2A2A2A] border-[#C400FF]/20 text-[#B0B0B0] hover:border-[#C400FF]/40 hover:text-[#E0E0E0]'
                    } disabled:opacity-50 disabled:cursor-not-allowed`}
                  >
                    {profile.charAt(0).toUpperCase() + profile.slice(1)}
                  </button>
                ))}
              </div>
            </div>
          )}

          {/* Thermal Safety Section */}
          {movementV2.thermal ? (
            <div className="bg-[#1A1A1A] border border-[#C400FF]/30 rounded-lg p-4 backdrop-blur-sm" style={{ boxShadow: '0 0 20px rgba(196, 0, 255, 0.1)' }}>
              <div className="flex items-center justify-between mb-2">
                <span className="text-[#B0B0B0] text-sm">Thermal Safety:</span>
                <span className={`font-medium capitalize ${getThermalColor(movementV2.thermal.state)}`}>
                  {movementV2.thermal.state || 'Unknown'}
                </span>
              </div>
              <div className="text-xs text-[#808080] space-y-1">
                <div className="flex justify-between">
                  <span>Max Temp Seen:</span>
                  <span className="text-[#E0E0E0]">
                    {movementV2.thermal.max_temp_seen?.toFixed(1) || '0.0'}°C
                  </span>
                </div>
                {movementV2.thermal.limits && (
                  <div className="flex justify-between">
                    <span>Limit:</span>
                    <span className="text-[#E0E0E0]">
                      {movementV2.thermal.limits.max_temp?.toFixed(1) || '0.0'}°C
                    </span>
                  </div>
                )}
              </div>
            </div>
          ) : (
            (() => {
              console.error('❌ [UI][V2] Missing thermal block in movementV2:', movementV2);
              return null;
            })()
          )}

          {/* Ramping Section */}
          {movementV2.ramping && (
            <div className="bg-[#1A1A1A] border border-[#C400FF]/30 rounded-lg p-4 backdrop-blur-sm" style={{ boxShadow: '0 0 20px rgba(196, 0, 255, 0.1)' }}>
              <div className="mb-2">
                <span className="text-[#B0B0B0] text-sm">Ramping:</span>
              </div>
              <div className="w-full h-2 bg-[#2A2A2A] rounded-full mt-2 mb-2 border border-[#C400FF]/20">
                <div
                  className="h-2 bg-gradient-to-r from-[#C400FF] to-[#8B00FF] rounded-full transition-all duration-300"
                  style={{ width: `${rampingPercent}%`, boxShadow: '0 0 10px rgba(196, 0, 255, 0.5)' }}
                />
              </div>
              <div className="flex justify-between text-xs text-[#808080]">
                <span>Current: <span className="text-[#E0E0E0]">{movementV2.ramping.current_pwm?.toFixed(0) || '0'}</span></span>
                <span>Target: <span className="text-[#E0E0E0]">{movementV2.ramping.target_pwm?.toFixed(0) || '0'}</span></span>
              </div>
              {movementV2.ramping.is_ramping && (
                <div className="text-xs text-[#C400FF] mt-1">Ramping in progress...</div>
              )}
            </div>
          )}

          {/* Watchdog Section */}
          {movementV2.watchdog && (
            <div className="bg-[#1A1A1A] border border-[#C400FF]/30 rounded-lg p-4 backdrop-blur-sm" style={{ boxShadow: '0 0 20px rgba(196, 0, 255, 0.1)' }}>
              <div className="flex items-center justify-between">
                <span className="text-[#B0B0B0] text-sm">Watchdog:</span>
                <span className="text-[#E0E0E0]">
                  {movementV2.watchdog.enabled
                    ? `${movementV2.watchdog.time_until_trigger?.toFixed(1) || '0.0'}s`
                    : 'Disabled'}
                </span>
              </div>
              {movementV2.watchdog.enabled && (
                <div className="text-xs text-[#808080] mt-1">
                  State: <span className="text-[#E0E0E0] capitalize">{movementV2.watchdog.state || 'active'}</span>
                </div>
              )}
            </div>
          )}

          {/* PID Status Section */}
          {movementV2.pid && (
            <div className="bg-[#1A1A1A] border border-[#C400FF]/30 rounded-lg p-4 backdrop-blur-sm" style={{ boxShadow: '0 0 20px rgba(196, 0, 255, 0.1)' }}>
              <div className="flex items-center justify-between mb-2">
                <span className="text-[#B0B0B0] text-sm">PID Control:</span>
                <span className={`font-medium ${movementV2.pid.enabled ? 'text-[#00FF88]' : 'text-[#808080]'}`}>
                  {movementV2.pid.enabled ? 'Enabled' : 'Disabled'}
                </span>
              </div>
              {movementV2.pid.enabled && movementV2.pid.tuning && (
                <div className="text-xs text-[#808080] space-y-1">
                  {movementV2.pid.target_rpm !== undefined && (
                    <div className="flex justify-between">
                      <span>Target RPM:</span>
                      <span className="text-[#E0E0E0]">{movementV2.pid.target_rpm.toFixed(1)}</span>
                    </div>
                  )}
                  <div className="flex justify-between">
                    <span>Kp:</span>
                    <span className="text-[#E0E0E0]">{movementV2.pid.tuning.kp.toFixed(3)}</span>
                  </div>
                  <div className="flex justify-between">
                    <span>Ki:</span>
                    <span className="text-[#E0E0E0]">{movementV2.pid.tuning.ki.toFixed(3)}</span>
                  </div>
                  <div className="flex justify-between">
                    <span>Kd:</span>
                    <span className="text-[#E0E0E0]">{movementV2.pid.tuning.kd.toFixed(3)}</span>
                  </div>
                  {movementV2.pid.tuning.kf !== undefined && movementV2.pid.tuning.kf !== 0 && (
                    <div className="flex justify-between">
                      <span>Kf:</span>
                      <span className="text-[#E0E0E0]">{movementV2.pid.tuning.kf.toFixed(3)}</span>
                    </div>
                  )}
                </div>
              )}
              {!movementV2.pid.enabled && movementV2.pid.available === false && (
                <div className="text-xs text-[#808080] mt-1">PID controller not available</div>
              )}
            </div>
          )}
        </div>

        {/* Exit Button */}
        <div className="flex justify-end p-6 pt-4 border-t border-[#C400FF]/30">
          <button
            onClick={onClose}
            className="px-6 py-2 bg-gradient-to-r from-[#4B0082] to-[#1A1A1A] hover:from-[#6B0082] hover:to-[#2A2A2A] text-[#E0E0E0] rounded-lg transition-all text-sm font-medium border border-[#C400FF]/40 hover:border-[#C400FF] hover:shadow-[0_0_15px_rgba(196,0,255,0.3)]"
          >
            Exit
          </button>
        </div>
      </div>
    </div>
  );

  // Use portal to render at document body level, escaping component hierarchy
  return typeof document !== 'undefined'
    ? createPortal(modalContent, document.body)
    : null;
};

export default MovementV2Modal;

