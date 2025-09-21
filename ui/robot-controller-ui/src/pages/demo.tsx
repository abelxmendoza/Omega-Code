import React, { useState } from 'react';
import CarControlPanel from '@/components/CarControlPanel';
import CameraControlPanel from '@/components/CameraControlPanel';
import LedControl from '@/components/LedControl';
import SensorDashboard from '@/components/SensorDashboard';
import SpeedControl from '@/components/SpeedControl';
import Status from '@/components/Status';
import ComponentWrapper from '@/components/ComponentWrapper';

const DemoPage: React.FC = () => {
  const [carSpeed, setCarSpeed] = useState(50);
  const [cameraTilt, setCameraTilt] = useState(0);
  const [cameraPan, setCameraPan] = useState(0);
  const [ledColor, setLedColor] = useState('#ff0000');
  const [ledBrightness, setLedBrightness] = useState(75);
  const [ledMode, setLedMode] = useState<'single' | 'multi'>('single');
  const [ledPattern, setLedPattern] = useState<'static' | 'pulse' | 'blink' | 'music'>('static');
  const [ledInterval, setLedInterval] = useState(1000);
  const [connected, setConnected] = useState(true);
  const [batteryLevel, setBatteryLevel] = useState(85);

  const handleCarCommand = (command: string) => {
    console.log('Car command:', command);
  };

  const handleCameraCommand = (command: string) => {
    console.log('Camera command:', command);
  };

  const mockSensors = {
    ultrasonic: { distance: 25, unit: 'cm' },
    temperature: { value: 22.5, unit: '°C' },
    humidity: { value: 45, unit: '%' },
    light: { value: 850, unit: 'lux' }
  };

  return (
    <div className="min-h-screen bg-gray-900 p-4">
      <div className="max-w-7xl mx-auto">
        <h1 className="text-3xl font-bold text-white mb-8 text-center">
          Robot Controller Demo
        </h1>

        {/* Status Bar */}
        <div className="mb-6">
          <ComponentWrapper title="System Status">
            <div className="flex flex-col sm:flex-row gap-4">
              <Status 
                connected={connected} 
                batteryLevel={batteryLevel}
                upCount={4}
                total={4}
              />
              <div className="flex gap-2">
                <button 
                  className="px-3 py-1 bg-blue-600 hover:bg-blue-700 text-white rounded text-sm"
                  onClick={() => setConnected(!connected)}
                >
                  Toggle Connection
                </button>
                <button 
                  className="px-3 py-1 bg-green-600 hover:bg-green-700 text-white rounded text-sm"
                  onClick={() => setBatteryLevel(Math.random() * 100)}
                >
                  Random Battery
                </button>
              </div>
            </div>
          </ComponentWrapper>
        </div>

        {/* Control Panels */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
          <ComponentWrapper>
            <CarControlPanel 
              onCommand={handleCarCommand}
              speed={carSpeed}
              onSpeedChange={setCarSpeed}
            />
          </ComponentWrapper>

          <ComponentWrapper>
            <CameraControlPanel 
              onCommand={handleCameraCommand}
              tilt={cameraTilt}
              pan={cameraPan}
              onTiltChange={setCameraTilt}
              onPanChange={setCameraPan}
            />
          </ComponentWrapper>
        </div>

        {/* LED and Speed Controls */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 mb-6">
          <ComponentWrapper>
            <LedControl 
              color={ledColor}
              brightness={ledBrightness}
              mode={ledMode}
              pattern={ledPattern}
              interval={ledInterval}
              onColorChange={setLedColor}
              onBrightnessChange={setLedBrightness}
              onModeChange={(mode) => setLedMode(mode as 'single' | 'multi')}
              onPatternChange={(pattern) => setLedPattern(pattern as any)}
              onIntervalChange={setLedInterval}
            />
          </ComponentWrapper>

          <ComponentWrapper>
            <SpeedControl 
              speed={carSpeed}
              onSpeedChange={setCarSpeed}
              label="Motor Speed"
            />
          </ComponentWrapper>
        </div>

        {/* Sensor Dashboard */}
        <ComponentWrapper>
          <SensorDashboard 
            sensors={mockSensors}
            onSensorUpdate={(sensorId, value) => {
              console.log('Sensor update:', sensorId, value);
            }}
          />
        </ComponentWrapper>

        {/* Debug Info */}
        <div className="mt-8 p-4 bg-gray-800 rounded-lg">
          <h3 className="text-white font-bold mb-4">Debug Information</h3>
          <div className="text-gray-300 text-sm space-y-2">
            <div>Car Speed: {carSpeed}%</div>
            <div>Camera Tilt: {cameraTilt}°</div>
            <div>Camera Pan: {cameraPan}°</div>
            <div>LED Color: {ledColor}</div>
            <div>LED Brightness: {ledBrightness}%</div>
            <div>LED Mode: {ledMode}</div>
            <div>LED Pattern: {ledPattern}</div>
            <div>LED Interval: {ledInterval}ms</div>
            <div>Connected: {connected ? 'Yes' : 'No'}</div>
            <div>Battery: {batteryLevel.toFixed(1)}%</div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DemoPage;
