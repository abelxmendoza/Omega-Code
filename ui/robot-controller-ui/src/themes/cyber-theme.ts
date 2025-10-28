/**
 * Omega Technologies Cyber-Industrial Theme
 * High-tech robotics control interface
 */

export const cyberTheme = {
  colors: {
    background: {
      primary: '#050007',
      secondary: '#0a0a0a',
      surface: '#151015',
      elevated: '#1a1a1a',
    },
    neon: {
      purple: '#C400FF',
      purpleDark: '#8B00FF',
      magenta: '#FF0048',
      cyan: '#00FFFF',
    },
    text: {
      primary: '#FFFFFF',
      secondary: '#E0E0E0',
      tertiary: '#B0B0B0',
      muted: '#808080',
    },
    status: {
      online: '#00FF88',
      warning: '#FFB800',
      error: '#FF0048',
      info: '#00B8FF',
    },
  },
  fonts: {
    display: "'Orbitron', 'Audiowide', sans-serif",
    primary: "'Rajdhani', 'Exo 2', sans-serif",
    mono: "'Fira Code', 'JetBrains Mono', monospace",
  },
} as const;

export type CyberTheme = typeof cyberTheme;

