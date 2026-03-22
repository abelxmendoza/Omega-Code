# Robot Controller UI Components

This document provides an overview of the React components available in the robot controller UI.

## Core Components

### CarControlPanel
Controls robot movement with speed adjustment and directional buttons.

**Props:**
- `onCommand?: (command: string) => void` - Callback for movement commands
- `speed?: number` - Current speed (0-100)
- `onSpeedChange?: (speed: number) => void` - Speed change callback

**Commands:** `forward`, `backward`, `left`, `right`, `stop`

### CameraControlPanel
Controls camera movement with tilt/pan controls and directional buttons.

**Props:**
- `onCommand?: (command: string) => void` - Callback for camera commands
- `tilt?: number` - Current tilt angle (-45 to 45)
- `pan?: number` - Current pan angle (-90 to 90)
- `onTiltChange?: (tilt: number) => void` - Tilt change callback
- `onPanChange?: (pan: number) => void` - Pan change callback

**Commands:** `up`, `down`, `left`, `right`, `reset`

### LedControl
Controls LED lighting with color, brightness, mode, and pattern settings.

**Props:**
- `color?: string` - LED color (hex)
- `brightness?: number` - Brightness (0-100)
- `mode?: 'single' | 'multi'` - Lighting mode
- `pattern?: 'static' | 'pulse' | 'blink' | 'music'` - Lighting pattern
- `interval?: number` - Pattern interval (ms)
- `onColorChange?: (color: string) => void` - Color change callback
- `onBrightnessChange?: (brightness: number) => void` - Brightness change callback
- `onModeChange?: (mode: string) => void` - Mode change callback
- `onPatternChange?: (pattern: string) => void` - Pattern change callback
- `onIntervalChange?: (interval: number) => void` - Interval change callback

### SensorDashboard
Displays sensor data in a grid layout with update functionality.

**Props:**
- `sensors?: Record<string, any>` - Sensor data object
- `onSensorUpdate?: (sensorId: string, value: any) => void` - Sensor update callback

### SpeedControl
Generic speed control component with slider and quick buttons.

**Props:**
- `speed?: number` - Current speed (0-100)
- `onSpeedChange?: (speed: number) => void` - Speed change callback
- `label?: string` - Control label (default: "Speed")
- `min?: number` - Minimum value (default: 0)
- `max?: number` - Maximum value (default: 100)

### Status
Displays connection status and battery level with visual indicators.

**Props:**
- `connected?: boolean` - Connection status
- `batteryLevel?: number` - Battery level (0-100)
- `battery?: number` - Battery level (test compatibility)
- `status?: string` - Status string ("Connected" | "Disconnected")
- `upCount?: number` - Services up count
- `total?: number` - Total services count

## Utility Components

### ErrorBoundary
Catches JavaScript errors in component tree and displays fallback UI.

**Props:**
- `children: ReactNode` - Child components
- `fallback?: ReactNode` - Custom fallback UI

### ComponentWrapper
Wraps components with error boundary and consistent styling.

**Props:**
- `children: ReactNode` - Child components
- `title?: string` - Optional title
- `className?: string` - Custom CSS classes

## Usage Examples

```tsx
import { CarControlPanel, LedControl, SensorDashboard } from '@/components';

function RobotDashboard() {
  const handleMovement = (command: string) => {
    console.log('Movement command:', command);
  };

  const handleSpeedChange = (speed: number) => {
    console.log('Speed changed to:', speed);
  };

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      <CarControlPanel 
        onCommand={handleMovement}
        onSpeedChange={handleSpeedChange}
        speed={75}
      />
      
      <LedControl 
        color="#ff0000"
        brightness={80}
        mode="single"
        pattern="pulse"
        onColorChange={(color) => console.log('Color:', color)}
      />
      
      <SensorDashboard 
        sensors={{
          ultrasonic: { distance: 25, unit: 'cm' },
          temperature: { value: 22.5, unit: '°C' }
        }}
      />
    </div>
  );
}
```

## Styling

All components use Tailwind CSS with a consistent dark theme:
- Background: `bg-gray-800`
- Text: `text-white`
- Buttons: `bg-blue-600 hover:bg-blue-700`
- Inputs: `bg-gray-700`
- Borders: `rounded-lg`
- Spacing: `p-4`, `space-y-4`, `gap-2`
