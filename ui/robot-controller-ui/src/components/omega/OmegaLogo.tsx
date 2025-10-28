import React from 'react';

interface OmegaLogoProps {
  size?: 'sm' | 'md' | 'lg';
  className?: string;
}

const OmegaLogo: React.FC<OmegaLogoProps> = ({ size = 'md', className = '' }) => {
  const sizeClasses = {
    sm: 'text-2xl',
    md: 'text-4xl',
    lg: 'text-6xl',
  };

  return (
    <div className={`font-display font-black tracking-widest ${sizeClasses[size]} ${className}`}>
      <span className="bg-gradient-to-r from-purple-400 to-cyan-300 bg-clip-text text-transparent animate-pulse-neon">
        Ω
      </span>
      <span className="text-white ml-2">OMEGA</span>
    </div>
  );
};

export default OmegaLogo;

