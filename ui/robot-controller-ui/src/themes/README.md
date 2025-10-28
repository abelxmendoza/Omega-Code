# Omega Technologies Theme System

A futuristic, cyberpunk-inspired design system for the Omega Technologies Robot Controller UI, blending industrial precision with digital aesthetics.

## 🎨 Design Philosophy

The Omega theme evokes a high-tech control deck for a next-gen AI robot. Think **"Matrix digital rain" meets "Armstrong robotics control panel"** with accents of glowing purple, metallic silver, and deep black.

### Core Aesthetic
- **Futuristic**: Sleek, high-tech, and immersive
- **Cyberpunk**: Dark backgrounds with neon glows
- **Industrial**: Clean, precise, and functional
- **Robotic**: Tactile, reactive, and responsive

## 🎨 Color Palette

### Primary Colors
- **Omega Black**: `#000000` - Primary background
- **Omega Dark Gray**: `#1A1A1A` - Secondary surfaces
- **Omega Purple**: `#4B0082` - Accent color
- **Omega Steel**: `#B0B0B0` - Text and borders

### Neon Accents
- **Neon Purple**: `#C400FF` - Primary glow
- **Neon Turquoise**: `#00FFFF` - Success states
- **Neon Blue**: `#0080FF` - Info states
- **Neon Pink**: `#FF0080` - Special accents

### Semantic Colors
- **Success**: `#00FF88` - Green
- **Warning**: `#FFAA00` - Amber
- **Error**: `#FF0066` - Red
- **Info**: `#00AAFF` - Blue

## 📦 Installation

The theme system is already integrated into the project. To use it:

### 1. Import Theme Styles

In your `globals.scss`, add:

```scss
@import '../styles/omega-theme.scss';
```

### 2. Apply Theme to Body

```tsx
<body className="omega-theme">
  {/* Your app */}
</body>
```

### 3. Use Omega Components

```tsx
import OmegaButton from '@/components/omega/OmegaButton';
import OmegaDashboardCard from '@/components/omega/OmegaDashboardCard';
import OmegaLogo from '@/components/omega/OmegaLogo';
```

## 🧩 Components

### OmegaButton

A futuristic button with neon glow effects and smooth animations.

```tsx
<OmegaButton 
  variant="primary"
  size="md"
  glow={true}
  onClick={() => console.log('Clicked!')}
>
  Execute Command
</OmegaButton>
```

**Variants**: `primary`, `secondary`, `success`, `danger`  
**Sizes**: `sm`, `md`, `lg`

### OmegaDashboardCard

A glassmorphism card panel for displaying telemetry and status.

```tsx
<OmegaDashboardCard 
  title="Motor Control"
  subtitle="RPM Monitoring"
  status="online"
  glow="purple"
>
  <div>Your content here</div>
</OmegaDashboardCard>
```

**Status**: `online`, `connecting`, `offline`, `idle`  
**Glow**: `purple`, `turquoise`, `blue`

### OmegaLogo

Animated logo with gradient text and glow effect.

```tsx
<OmegaLogo size="md" />
```

**Sizes**: `sm`, `md`, `lg`

### DigitalRain

Matrix-style digital rain background effect.

```tsx
<DigitalRain 
  opacity={0.03}
  speed={50}
  color="#00FF88"
/>
```

## 🎯 Tailwind Utilities

The theme extends Tailwind with custom utilities:

### Colors

```tsx
<div className="bg-omega-black text-neon-purple border-neon-turquoise">
  // Use omega colors
</div>
```

Available colors:
- `omega-black`, `omega-black-800`, `omega-purple`
- `neon-purple`, `neon-turquoise`, `neon-blue`
- `semantic-success`, `semantic-warning`, `semantic-error`

### Glassmorphism

```tsx
<div className="bg-glass-frost backdrop-blur-omega">
  Glass panel
</div>
```

### Shadows & Glows

```tsx
<div className="shadow-neon-purple hover:shadow-neon-turquoise">
  Neon glow effect
</div>
```

### Gradients

```tsx
<div className="bg-omega-purple-radial">
  Purple radial gradient
</div>

<div className="bg-omega-cyber">
  Cyber diagonal gradient
</div>
```

### Animations

```tsx
<div className="animate-pulse-neon">
  Pulsing neon glow
</div>

<div className="animate-glow-slow">
  Slow glow animation
</div>

<div className="animate-float">
  Floating animation
</div>
```

### Fonts

```tsx
<h1 className="font-display">Futuristic Display Text</h1>
<p className="font-omega">Omega Primary Font</p>
<code className="font-mono">Monospace Code</code>
```

## 🎨 Custom Styling

### Applying Neon Glows

```css
.my-element {
  box-shadow: 0 0 10px #C400FF, 0 0 20px #C400FF, 0 0 30px #C400FF;
}

.my-element:hover {
  box-shadow: 0 0 20px #C400FF, 0 0 30px #C400FF, 0 0 40px #C400FF;
}
```

### Glassmorphism Panels

```css
.glass-panel {
  background: rgba(26, 26, 26, 0.7);
  border: 1px solid rgba(196, 0, 255, 0.2);
  backdrop-filter: blur(10px);
  border-radius: 12px;
}
```

### Animated Borders

```tsx
<div className="omega-border-animated">
  Animated border effect
</div>
```

## 📱 Responsive Design

The theme includes responsive typography:

```scss
@media (max-width: 768px) {
  .omega-logo {
    font-size: 1.25rem;
  }
}
```

## 🚀 Example Usage

```tsx
import React from 'react';
import { DigitalRain } from '@/components/omega/DigitalRain';
import { OmegaLogo } from '@/components/omega/OmegaLogo';
import { OmegaButton } from '@/components/omega/OmegaButton';
import { OmegaDashboardCard } from '@/components/omega/OmegaDashboardCard';

export default function RobotControlPanel() {
  return (
    <div className="omega-theme min-h-screen">
      <DigitalRain opacity={0.03} />
      
      <div className="omega-content relative z-10 p-8">
        <header className="mb-8">
          <OmegaLogo size="lg" />
        </header>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          <OmegaDashboardCard 
            title="Motor Status"
            status="online"
            glow="purple"
          >
            <p className="text-2xl font-mono">2,450 RPM</p>
          </OmegaDashboardCard>

          <OmegaDashboardCard 
            title="Ultrasonic Sensor"
            status="connecting"
            glow="turquoise"
          >
            <p className="text-2xl font-mono">12.5 cm</p>
          </OmegaDashboardCard>

          <OmegaDashboardCard 
            title="System Health"
            status="online"
            glow="blue"
          >
            <p className="text-2xl font-mono">98%</p>
          </OmegaDashboardCard>
        </div>

        <div className="mt-8 flex gap-4">
          <OmegaButton variant="primary" glow={true}>
            Start Robot
          </OmegaButton>
          <OmegaButton variant="success" glow={true}>
            Calibrate
          </OmegaButton>
          <OmegaButton variant="danger" glow={true}>
            Emergency Stop
          </OmegaButton>
        </div>
      </div>
    </div>
  );
}
```

## 🔧 Customization

### Changing Primary Colors

Edit `tailwind.config.ts`:

```ts
colors: {
  neon: {
    purple: '#YOUR_COLOR',  // Change primary glow
  }
}
```

### Adjusting Glow Intensity

```ts
boxShadow: {
  'neon-purple': '0 0 5px #C400FF, 0 0 10px #C400FF, 0 0 15px #C400FF',
}
```

### Modifying Glassmorphism

```scss
.omega-panel {
  background: rgba(26, 26, 26, 0.5);  // Adjust opacity
  backdrop-filter: blur(5px);          // Adjust blur
}
```

## 📚 Typography

The theme uses these Google Fonts:
- **Orbitron**: Display text (futuristic, bold)
- **Rajdhani**: Body text (clean, modern)
- **Exo 2**: Alternative body (technical)
- **Fira Code**: Monospace (code, telemetry)

All fonts are loaded via Google Fonts CDN in the theme stylesheet.

## 🎯 Best Practices

1. **Use glassmorphism** for panels and cards
2. **Apply neon glows** sparingly on interactive elements
3. **Maintain contrast** between text and backgrounds
4. **Use semantic colors** for status indicators
5. **Keep animations subtle** - avoid overwhelming the UI
6. **Leverage the grid** for structured layouts

## 🐛 Troubleshooting

### Glows not showing

Make sure the element has a background or border that the glow can reflect against.

### Fonts not loading

Check that Google Fonts are being imported in `omega-theme.scss`.

### Animation performance

Reduce animation intensity on low-end devices by adjusting animation duration.

## 📄 License

Part of the Omega Technologies Robot Controller project.

