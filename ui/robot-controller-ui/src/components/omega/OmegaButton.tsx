import React from 'react';

interface OmegaButtonProps {
  children: React.ReactNode;
  onClick?: () => void;
  variant?: 'primary' | 'secondary' | 'success' | 'danger';
  size?: 'sm' | 'md' | 'lg';
  disabled?: boolean;
  glow?: boolean;
  className?: string;
  type?: 'button' | 'submit' | 'reset';
}

const OmegaButton: React.FC<OmegaButtonProps> = ({
  children,
  onClick,
  variant = 'primary',
  size = 'md',
  disabled = false,
  glow = true,
  className = '',
  type = 'button',
}) => {
  const sizeClasses = {
    sm: 'px-3 py-1.5 text-sm',
    md: 'px-6 py-3 text-base',
    lg: 'px-8 py-4 text-lg',
  };

  const variantClasses = {
    primary: 'bg-purple-500/20 border-purple-500 text-purple-300',
    secondary: 'bg-steel-dark/20 border-steel text-steel-light',
    success: 'bg-green-500/20 border-green-500 text-green-300',
    danger: 'bg-red-500/20 border-red-500 text-red-300',
  };

  const glowClass = glow ? 'hover:shadow-neon-purple' : '';

  return (
    <button
      type={type}
      onClick={onClick}
      disabled={disabled}
      className={`
        font-omega font-semibold
        rounded-omega-sm
        border-2
        transition-all duration-300
        relative overflow-hidden
        ${sizeClasses[size]}
        ${variantClasses[variant]}
        ${glowClass}
        ${disabled ? 'opacity-50 cursor-not-allowed' : 'cursor-pointer'}
        ${className}
      `}
    >
      {/* Animated shimmer effect */}
      {!disabled && (
        <span className="absolute inset-0 -translate-x-full animate-shimmer bg-gradient-to-r from-transparent via-white/10 to-transparent" />
      )}
      <span className="relative z-10">{children}</span>
    </button>
  );
};

export default OmegaButton;

