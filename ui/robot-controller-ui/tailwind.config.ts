// File: tailwind.config.ts
// Summary:
// Configures TailwindCSS with Omega Technologies theme system - a futuristic,
// cyberpunk-inspired design with neon glows, glassmorphism, and industrial aesthetics.

import type { Config } from "tailwindcss";

const config: Config = {
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}", // Pages directory
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}", // Components directory
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}", // App directory for Next.js 13+
    "./src/themes/**/*.{js,ts}", // Theme files
  ],
  theme: {
    extend: {
      // Omega Theme Colors
      colors: {
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
        neon: {
          purple: '#C400FF',
          'purple-soft': '#D400FF',
          turquoise: '#00FFFF',
          'turquoise-soft': '#00DDDD',
          blue: '#0080FF',
          pink: '#FF0080',
        },
        semantic: {
          success: '#00FF88',
          warning: '#FFAA00',
          error: '#FF0066',
          info: '#00AAFF',
        },
      },
      
      // Glassmorphism backgrounds
      backgroundColor: {
        'glass-frost': 'rgba(26, 26, 26, 0.7)',
        'glass-dark': 'rgba(0, 0, 0, 0.8)',
        'glass-purple': 'rgba(75, 0, 130, 0.3)',
        'glass-neon': 'rgba(196, 0, 255, 0.1)',
      },

      // Custom Gradients
      backgroundImage: {
        "gradient-radial": "radial-gradient(var(--tw-gradient-stops))",
        "gradient-conic": "conic-gradient(from 180deg at 50% 50%, var(--tw-gradient-stops))",
        "omega-purple-radial": "radial-gradient(circle at center, rgba(196, 0, 255, 0.1) 0%, rgba(0, 0, 0, 0.8) 100%)",
        "omega-cyber": "linear-gradient(135deg, #4B0082 0%, #1A1A1A 50%, #000000 100%)",
        "omega-neon-border": "linear-gradient(90deg, transparent, #C400FF, transparent)",
        "omega-steel": "linear-gradient(180deg, #2A2A2A 0%, #1A1A1A 100%)",
      },

      // Custom Shadows (Neon Glows)
      boxShadow: {
        'neon-purple': '0 0 10px #C400FF, 0 0 20px #C400FF, 0 0 30px #C400FF',
        'neon-purple-soft': '0 0 5px #C400FF, 0 0 10px #C400FF',
        'neon-turquoise': '0 0 10px #00FFFF, 0 0 20px #00FFFF, 0 0 30px #00FFFF',
        'neon-blue': '0 0 10px #0080FF, 0 0 20px #0080FF',
        'glass-outset': '0 8px 32px rgba(0, 0, 0, 0.4)',
      },

      // Custom Fonts
      fontFamily: {
        'omega': ['Orbitron', 'Rajdhani', 'Exo 2', 'Inter', 'sans-serif'],
        'mono': ['Fira Code', 'JetBrains Mono', 'Courier New', 'monospace'],
        'display': ['Orbitron', 'sans-serif'],
      },

      // Custom Animations
      keyframes: {
        'pulse-neon': {
          '0%, 100%': { 
            boxShadow: '0 0 5px #C400FF, 0 0 10px #C400FF, 0 0 15px #C400FF',
            opacity: '1',
          },
          '50%': { 
            boxShadow: '0 0 10px #C400FF, 0 0 20px #C400FF, 0 0 30px #C400FF, 0 0 40px #C400FF',
            opacity: '0.8',
          },
        },
        'glow-slow': {
          '0%, 100%': { opacity: '1' },
          '50%': { opacity: '0.6' },
        },
        'float': {
          '0%, 100%': { transform: 'translateY(0)' },
          '50%': { transform: 'translateY(-10px)' },
        },
        'cyber-rain': {
          '0%': { transform: 'translateY(-100%)' },
          '100%': { transform: 'translateY(100vh)' },
        },
        'scan-line': {
          '0%': { transform: 'translateX(-100%)' },
          '100%': { transform: 'translateX(100%)' },
        },
      },
      animation: {
        'pulse-neon': 'pulse-neon 2s ease-in-out infinite',
        'glow-slow': 'glow-slow 1.5s ease-in-out infinite',
        'float': 'float 3s ease-in-out infinite',
        'cyber-rain': 'cyber-rain linear infinite',
        'scan-line': 'scan-line 3s linear infinite',
      },

      // Custom Border Radius
      borderRadius: {
        'omega': '12px',
        'omega-sm': '8px',
        'omega-lg': '16px',
      },

      // Custom Backdrop Blur
      backdropBlur: {
        'omega': '10px',
        'proxy': '20px',
      },
    },
  },
  plugins: [],
};

export default config;
