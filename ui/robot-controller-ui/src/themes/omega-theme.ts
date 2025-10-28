/**
 * Omega Technologies Theme Configuration
 * 
 * A futuristic, cyberpunk-inspired design system blending industrial precision
 * with digital aesthetics. Think "Matrix digital rain" meets "AI control room."
 * 
 * Core Palette:
 * - Deep black (#000000) - Primary background
 * - Dark purple (#4B0082) - Secondary surface
 * - Steel gray (#1A1A1A, #B0B0B0) - Borders and accents
 * - Neon purple (#C400FF) - Interactive elements
 * - Turquoise glow (#00FFFF) - Success states
 * 
 * Typography: Orbitron, Rajdhani, or Exo 2 for futuristic feel
 */

export const omegaTheme = {
  colors: {
    // Core Palette
    omega: {
      black: '#000000',
      'black-900': '#0A0A0A',
      'black-800': '#1A1A1A',
      'black-700': '#2A2A2A',
      purple: '#4B0082',
      'purple-light': '#6B0082',
      'purple-dark': '#3A0052',
      steel: '#B0B0B0',
      'steel-dark': '#808080',
      'steel-light': '#D0D0D0',
    },

    // Neon & Glow
    neon: {
      purple: '#C400FF',
      'purple-soft': '#D400FF',
      turquoise: '#00FFFF',
      'turquoise-soft': '#00DDDD',
      blue: '#0080FF',
      pink: '#FF0080',
    },

    // Semantic Colors
    semantic: {
      success: '#00FF88',
      warning: '#FFAA00',
      error: '#FF0066',
      info: '#00AAFF',
    },

    // Glassmorphism (RGBA)
    glass: {
      'frost': 'rgba(26, 26, 26, 0.7)',
      'dark': 'rgba(0, 0, 0, 0.8)',
      'purple-tint': 'rgba(75, 0, 130, 0.3)',
      'neon-tint': 'rgba(196, 0, 255, 0.1)',
    },

    // Status Colors
    status: {
      online: '#00FF88',
      connecting: '#FFAA00',
      offline: '#FF0066',
      idle: '#B0B0B0',
    },
  },

  fonts: {
    primary: "'Orbitron', 'Rajdhani', 'Exo 2', 'Inter', sans-serif",
    mono: "'Fira Code', 'JetBrains Mono', 'Courier New', monospace",
    display: "'Orbitron', sans-serif",
  },

  shadows: {
    // Neon Glows
    'neon-purple': '0 0 10px #C400FF, 0 0 20px #C400FF, 0 0 30px #C400FF',
    'neon-purple-soft': '0 0 5px #C400FF, 0 0 10px #C400FF',
    'neon-turquoise': '0 0 10px #00FFFF, 0 0 20px #00FFFF, 0 0 30px #00FFFF',
    'neon-blue': '0 0 10px #0080FF, 0 0 20px #0080FF',
    
    // Glassmorphism
    'glass-inset': 'inset 0 2px 4px rgba(0, 0, 0, 0.5)',
    'glass-outset': '0 8px 32px rgba(0, 0, 0, 0.4)',
    
    // Depth
    'elevation-1': '0 2px 8px rgba(0, 0, 0, 0.3)',
    'elevation-2': '0 4px 16px rgba(0, 0, 0, 0.4)',
    'elevation-3': '0 8px 32px rgba(0, 0, 0, 0.5)',
  },

  gradients: {
    'purple-radial': 'radial-gradient(circle at center, rgba(196, 0, 255, 0.1) 0%, rgba(0, 0, 0, 0.8) 100%)',
    'cyber-diagonal': 'linear-gradient(135deg, #4B0082 0%, #1A1A1A 50%, #000000 100%)',
    'neon-border': 'linear-gradient(90deg, transparent, #C400FF, transparent)',
    'steel-surface': 'linear-gradient(180deg, #2A2A2A 0%, #1A1A1A 100%)',
  },

  animations: {
    pulse: {
      duration: '2s',
      timing: 'ease-in-out',
    },
    glow: {
      duration: '1.5s',
      timing: 'ease-in-out',
    },
    float: {
      duration: '3s',
      timing: 'ease-in-out',
    },
  },

  spacing: {
    panel: {
      padding: '1.5rem',
      gap: '1rem',
      borderRadius: '12px',
    },
    card: {
      padding: '1rem',
      gap: '0.75rem',
      borderRadius: '8px',
    },
  },

  opacity: {
    disabled: 0.3,
    hover: 0.8,
    active: 0.6,
    glass: 0.7,
  },
} as const;

/**
 * CSS Variables for Tailwind Integration
 */
export const omegaThemeCssVars = `
  --omega-black: ${omegaTheme.colors.omega.black};
  --omega-black-800: ${omegaTheme.colors.omega['black-800']};
  --omega-purple: ${omegaTheme.colors.omega.purple};
  --omega-neon-purple: ${omegaTheme.colors.neon.purple};
  --omega-neon-turquoise: ${omegaTheme.colors.neon.turquoise};
  
  --omega-shadow-neon-purple: ${omegaTheme.shadows['neon-purple']};
  --omega-shadow-neon-turquoise: ${omegaTheme.shadows['neon-turquoise']};
`;

/**
 * Tailwind Theme Extension Type
 */
export type OmegaTheme = typeof omegaTheme;

/**
 * Helper function to generate glow classes
 */
export const getGlowColor = (color: 'purple' | 'turquoise' | 'blue' | 'pink' | 'soft-purple' = 'purple'): string => {
  const glowMap = {
    purple: omegaTheme.shadows['neon-purple'],
    'soft-purple': omegaTheme.shadows['neon-purple-soft'],
    turquoise: omegaTheme.shadows['neon-turquoise'],
    blue: omegaTheme.shadows['neon-blue'],
    pink: omegaTheme.shadows['neon-purple'],
  };
  return glowMap[color];
};

/**
 * Helper function for glassmorphism backgrounds
 */
export const getGlassBg = (tint: 'frost' | 'dark' | 'purple' | 'neon' = 'frost'): string => {
  const glassMap = {
    frost: omegaTheme.colors.glass.frost,
    dark: omegaTheme.colors.glass.dark,
    purple: omegaTheme.colors.glass['purple-tint'],
    neon: omegaTheme.colors.glass['neon-tint'],
  };
  return glassMap[tint];
};

