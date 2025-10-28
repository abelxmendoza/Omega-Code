import React, { useState, useEffect } from 'react';

export default function CyberDemoPage() {
  const [currentTime, setCurrentTime] = useState('');

  useEffect(() => {
    const updateTime = () => setCurrentTime(new Date().toLocaleTimeString());
    updateTime();
    const interval = setInterval(updateTime, 1000);
    return () => clearInterval(interval);
  }, []);

  return (
    <div className="cyber-theme min-h-screen">
      <div className="p-8">
        <header className="mb-12 flex items-center justify-between">
          <div>
            <div className="cyber-logo text-5xl mb-4">OMEGA</div>
            <p className="text-gray-400 font-mono text-lg">ROBOT CONTROL SYSTEM v2.0</p>
          </div>
          <div className="flex items-center gap-4">
            <div className="status-online text-sm font-mono">● ONLINE</div>
            <div className="w-32 h-2 bg-black rounded-full overflow-hidden">
              <div className="h-full bg-green-500" style={{ width: '87%' }} />
            </div>
            <div className="text-gray-500 font-mono text-xs">87%</div>
          </div>
        </header>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-6 mb-8">
          <div className="cyber-panel p-6">
            <h3 className="text-white text-xl mb-4 font-display">Motor Control</h3>
            <div className="text-center">
              <p className="text-5xl font-mono text-cyan-400 mb-2">2,450</p>
              <p className="text-gray-400">RPM</p>
            </div>
          </div>

          <div className="cyber-panel p-6">
            <h3 className="text-white text-xl mb-4 font-display">Ultrasonic</h3>
            <div className="text-center">
              <p className="text-5xl font-mono text-green-400 mb-2">12.5</p>
              <p className="text-gray-400">cm</p>
            </div>
          </div>

          <div className="cyber-panel p-6">
            <h3 className="text-white text-xl mb-4 font-display">Battery</h3>
            <div className="text-center">
              <p className="text-5xl font-mono text-purple-400 mb-2">87%</p>
              <p className="text-gray-400">CHARGED</p>
            </div>
          </div>
        </div>

        <div className="cyber-panel p-8 mb-8">
          <h2 className="text-white text-2xl mb-6 font-display">ROBOT CONTROL</h2>
          <div className="grid grid-cols-3 gap-6 mb-6">
            <div>
              <label className="block text-gray-400 mb-2 font-mono text-sm">Speed</label>
              <input type="range" min="0" max="100" defaultValue="50" className="w-full" />
            </div>
            <div>
              <label className="block text-gray-400 mb-2 font-mono text-sm">Range</label>
              <input type="range" min="0" max="400" defaultValue="200" className="w-full" />
            </div>
            <div>
              <label className="block text-gray-400 mb-2 font-mono text-sm">Mode</label>
              <div className="flex gap-2">
                <button className="cyber-button flex-1">ENABLE</button>
                <button className="cyber-button flex-1">MANUAL</button>
              </div>
            </div>
          </div>
        </div>

        <div className="flex flex-wrap gap-4">
          <button className="cyber-button">▶ START</button>
          <button className="cyber-button">CALIBRATE</button>
          <button className="cyber-button">DIAGNOSTIC</button>
          <button className="cyber-button">⚠ STOP</button>
        </div>

        <footer className="mt-12 pt-6 border-t border-purple-500/30">
          <div className="flex justify-between text-sm font-mono text-gray-400">
            <div>
              <span className="status-online">● SYSTEM ONLINE</span>
            </div>
            <div>{currentTime || '--:--:--'}</div>
          </div>
        </footer>
      </div>
    </div>
  );
}

