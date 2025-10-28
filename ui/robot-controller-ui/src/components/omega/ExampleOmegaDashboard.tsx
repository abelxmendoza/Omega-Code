/**
 * Example Omega Dashboard
 * Demonstrates the Omega theme system with a complete dashboard layout
 */

import React from 'react';
import { OmegaButton, OmegaDashboardCard, OmegaLogo, DigitalRain } from './index';

const ExampleOmegaDashboard: React.FC = () => {
  return (
    <div className="omega-theme min-h-screen bg-omega-black">
      {/* Digital Rain Background */}
      <DigitalRain opacity={0.03} speed={50} color="#00FF88" />
      
      {/* Main Content */}
      <div className="omega-content relative z-10 p-8">
        {/* Header */}
        <header className="mb-8 flex items-center justify-between">
          <div>
            <OmegaLogo size="lg" />
            <p className="text-omega-steel mt-2 font-mono text-sm">
              NEXT-GEN AI ROBOT CONTROL SYSTEM v2.0
            </p>
          </div>
          <div className="flex items-center gap-4">
            <div className="text-semantic-success text-sm font-mono">
              ● ONLINE
            </div>
            <div className="w-32 h-2 bg-omega-black-800 rounded-full overflow-hidden">
              <div className="h-full bg-semantic-success" style={{ width: '87%' }} />
            </div>
          </div>
        </header>

        {/* Sensor Grid */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
          <OmegaDashboardCard 
            title="Motor Control"
            subtitle="Left Drive Motor"
            status="online"
            glow="purple"
          >
            <div className="text-center">
              <p className="text-4xl font-mono font-bold text-neon-purple">2,450</p>
              <p className="text-sm text-omega-steel mt-2">RPM</p>
            </div>
          </OmegaDashboardCard>

          <OmegaDashboardCard 
            title="Ultrasonic Sensor"
            subtitle="Distance Measurement"
            status="online"
            glow="turquoise"
          >
            <div className="text-center">
              <p className="text-4xl font-mono font-bold text-neon-turquoise">12.5</p>
              <p className="text-sm text-omega-steel mt-2">cm</p>
            </div>
          </OmegaDashboardCard>

          <OmegaDashboardCard 
            title="Battery Status"
            subtitle="Power Level"
            status="online"
            glow="blue"
          >
            <div className="text-center">
              <p className="text-4xl font-mono font-bold text-neon-blue">87%</p>
              <p className="text-sm text-omega-steel mt-2">CHARGED</p>
            </div>
          </OmegaDashboardCard>

          <OmegaDashboardCard 
            title="Network Status"
            subtitle="Connection Quality"
            status="connecting"
            glow="purple"
          >
            <div className="text-center">
              <p className="text-4xl font-mono font-bold text-semantic-warning">---</p>
              <p className="text-sm text-omega-steel mt-2">SYNCING</p>
            </div>
          </OmegaDashboardCard>
        </div>

        {/* Control Panel */}
        <div className="mb-8">
          <h2 className="font-display text-2xl font-bold text-white mb-4">
            ROBOT CONTROL
          </h2>
          
          <div className="omega-panel bg-glass-frost border-neon-purple/20 p-6">
            <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
              <div>
                <label className="block text-omega-steel text-sm mb-2 font-mono uppercase">
                  Movement Speed
                </label>
                <input 
                  type="range" 
                  min="0" 
                  max="100" 
                  defaultValue="50"
                  className="w-full h-2 bg-omega-black-800 rounded-lg appearance-none cursor-pointer"
                />
                <div className="flex justify-between text-xs text-omega-steel mt-1">
                  <span>0%</span>
                  <span>50%</span>
                  <span>100%</span>
                </div>
              </div>

              <div>
                <label className="block text-omega-steel text-sm mb-2 font-mono uppercase">
                  Sensor Range
                </label>
                <input 
                  type="range" 
                  min="10" 
                  max="400" 
                  defaultValue="200"
                  className="w-full h-2 bg-omega-black-800 rounded-lg appearance-none cursor-pointer"
                />
                <div className="flex justify-between text-xs text-omega-steel mt-1">
                  <span>10cm</span>
                  <span>200cm</span>
                  <span>400cm</span>
                </div>
              </div>

              <div>
                <label className="block text-omega-steel text-sm mb-2 font-mono uppercase">
                  Auto Mode
                </label>
                <div className="flex items-center gap-4">
                  <button className="omega-button omega-button-primary flex-1">
                    ENABLE
                  </button>
                  <button className="omega-button omega-button-secondary flex-1">
                    MANUAL
                  </button>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Action Buttons */}
        <div className="flex flex-wrap gap-4">
          <OmegaButton variant="primary" glow={true} size="lg">
            ▶ START ROBOT
          </OmegaButton>
          
          <OmegaButton variant="success" glow={true} size="lg">
            CALIBRATE SENSORS
          </OmegaButton>
          
          <OmegaButton variant="secondary" glow={false} size="lg">
            DIAGNOSTIC MODE
          </OmegaButton>
          
          <OmegaButton variant="danger" glow={true} size="lg">
            ⚠ EMERGENCY STOP
          </OmegaButton>
        </div>

        {/* Footer Status */}
        <footer className="mt-12 pt-6 border-t border-omega-purple/20">
          <div className="flex items-center justify-between text-xs font-mono text-omega-steel">
            <div className="flex gap-6">
              <span className="text-semantic-success">● MOVEMENT: ONLINE</span>
              <span className="text-semantic-success">● ULTRASONIC: ONLINE</span>
              <span className="text-semantic-warning">● LIGHTING: CONNECTING</span>
            </div>
            <div>
              SYSTEM TIME: {new Date().toLocaleTimeString()}
            </div>
          </div>
        </footer>
      </div>
    </div>
  );
};

export default ExampleOmegaDashboard;

