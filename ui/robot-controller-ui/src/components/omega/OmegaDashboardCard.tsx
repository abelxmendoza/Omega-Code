/**
 * Omega Dashboard Card Component
 * 
 * A futuristic card component for displaying telemetry, status, and control elements
 * with neon purple accents and glassmorphism effects.
 */

import React from 'react';

interface OmegaDashboardCardProps {
  title: string;
  subtitle?: string;
  children: React.ReactNode;
  status?: 'online' | 'connecting' | 'offline' | 'idle';
  glow?: 'purple' | 'turquoise' | 'blue';
  className?: string;
}

const OmegaDashboardCard: React.FC<OmegaDashboardCardProps> = ({
  title,
  subtitle,
  children,
  status,
  glow = 'purple',
  className = '',
}) => {
  const glowClass = `shadow-neon-${glow}`;
  const statusColor =
    status === 'online' ? 'text-semantic-success' :
    status === 'connecting' ? 'text-semantic-warning' :
    status === 'offline' ? 'text-semantic-error' :
    'text-omega-steel';

  return (
    <div className={`
      omega-panel 
      bg-glass-frost
      border border-neon-purple/20
      backdrop-blur-omega
      p-6
      transition-all duration-300
      hover:border-neon-purple
      hover:shadow-neon-purple
      ${className}
    `}>
      {/* Header */}
      <div className="flex items-center justify-between mb-4">
        <div>
          <h3 className="font-display text-lg font-bold text-white">
            {title}
          </h3>
          {subtitle && (
            <p className="text-sm text-omega-steel mt-1">{subtitle}</p>
          )}
        </div>
        {status && (
          <div className={`text-xs font-mono uppercase ${statusColor}`}>
            {status}
          </div>
        )}
      </div>

      {/* Content */}
      <div className="text-omega-steel-light">
        {children}
      </div>
    </div>
  );
};

export default OmegaDashboardCard;

