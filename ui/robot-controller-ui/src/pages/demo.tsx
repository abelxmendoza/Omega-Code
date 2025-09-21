import React, { useState } from 'react';
import CarControlPanel from '@/components/CarControlPanel';
import CameraControlPanel from '@/components/CameraControlPanel';
import LedControl from '@/components/LedControl';
import SensorDashboard from '@/components/SensorDashboard';
import SensorVisualization from '@/components/SensorVisualization';
import SpeedControl from '@/components/SpeedControl';
import Status from '@/components/Status';
import ComponentWrapper from '@/components/ComponentWrapper';
import AIVision from '@/components/AIVision';
import { useWebSocket } from '@/hooks/useWebSocket';
import { useVoiceControl } from '@/hooks/useVoiceControl';

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

  // WebSocket integration
  const { data: wsData, isConnected: wsConnected, sendMessage } = useWebSocket();

  // Voice control
  const { 
    isListening, 
    isSupported: voiceSupported, 
    transcript, 
    error: voiceError,
    toggleListening 
  } = useVoiceControl({
    onCommand: (command) => {
      console.log('Voice command:', command);
      handleCarCommand(command);
    }
  });

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
                connected={wsConnected || connected} 
                batteryLevel={wsData.status?.battery || batteryLevel}
                upCount={4}
                total={4}
              />
              <div className="flex gap-2">
                <button 
                  className="px-3 py-1 bg-blue-600 hover:bg-blue-700 text-white rounded text-sm transition-all duration-200 hover:scale-105"
                  onClick={() => setConnected(!connected)}
                >
                  Toggle Connection
                </button>
                <button 
                  className="px-3 py-1 bg-green-600 hover:bg-green-700 text-white rounded text-sm transition-all duration-200 hover:scale-105"
                  onClick={() => setBatteryLevel(Math.random() * 100)}
                >
                  Random Battery
                </button>
                {voiceSupported && (
                  <button 
                    className={`px-3 py-1 rounded text-sm transition-all duration-200 hover:scale-105 ${
                      isListening 
                        ? 'bg-red-600 hover:bg-red-700 text-white' 
                        : 'bg-purple-600 hover:bg-purple-700 text-white'
                    }`}
                    onClick={toggleListening}
                  >
                    {isListening ? '🎤 Stop Voice' : '🎤 Voice Control'}
                  </button>
                )}
              </div>
            </div>
            {isListening && (
              <div className="mt-3 p-2 bg-purple-900/30 rounded text-purple-200 text-sm">
                🎤 Listening... Say &quot;forward&quot;, &quot;backward&quot;, &quot;left&quot;, &quot;right&quot;, or &quot;stop&quot;
              </div>
            )}
            {transcript && (
              <div className="mt-2 p-2 bg-blue-900/30 rounded text-blue-200 text-sm">
                📝 Heard: &quot;{transcript}&quot;
              </div>
            )}
            {voiceError && (
              <div className="mt-2 p-2 bg-red-900/30 rounded text-red-200 text-sm">
                ❌ Voice Error: {voiceError}
              </div>
            )}
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

        {/* Advanced Sensor Visualizations */}
        <div className="mb-6">
          <h2 className="text-2xl font-bold text-white mb-4">Advanced Sensor Visualizations</h2>
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
            <SensorVisualization
              sensorId="ultrasonic"
              data={{
                value: 25,
                unit: 'cm',
                timestamp: Date.now(),
                min: 0,
                max: 200
              }}
              type="gauge"
              color="#10b981"
              size="medium"
              showHistory={true}
            />
            <SensorVisualization
              sensorId="temperature"
              data={{
                value: 22.5,
                unit: '°C',
                timestamp: Date.now(),
                min: -10,
                max: 50
              }}
              type="line"
              color="#f59e0b"
              size="medium"
              showHistory={true}
            />
            <SensorVisualization
              sensorId="humidity"
              data={{
                value: 45,
                unit: '%',
                timestamp: Date.now(),
                min: 0,
                max: 100
              }}
              type="radial"
              color="#3b82f6"
              size="medium"
              showHistory={true}
            />
            <SensorVisualization
              sensorId="light"
              data={{
                value: 850,
                unit: 'lux',
                timestamp: Date.now(),
                min: 0,
                max: 1000
              }}
              type="bar"
              color="#8b5cf6"
              size="medium"
              showHistory={true}
            />
          </div>
        </div>

        {/* AI Computer Vision */}
        <div className="mb-6">
          <h2 className="text-2xl font-bold text-white mb-4">AI Computer Vision</h2>
          <ComponentWrapper>
            <AIVision 
              onObjectDetected={(objects) => {
                console.log('Objects detected:', objects);
              }}
              onFaceRecognized={(faces) => {
                console.log('Faces recognized:', faces);
              }}
              onMotionDetected={(motion) => {
                console.log('Motion detected:', motion);
              }}
              enabled={true}
              confidence={0.7}
            />
          </ComponentWrapper>
        </div>

        {/* Basic Sensor Dashboard */}
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
