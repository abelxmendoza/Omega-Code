/*
# File: /src/components/sensors/UltrasonicVisualization.tsx
# Summary:
#   Radar-style visual display of ultrasonic sensor data showing:
#   - Circular radar display with distance rings
#   - Sweeping beam animation
#   - Target blips at detected obstacles
#   - Real-time distance readings
*/

'use client';

import React, { useEffect, useRef, useState } from 'react';
import { Radar, AlertTriangle } from 'lucide-react';

interface UltrasonicData {
  distance_cm: number;
  distance_m: number;
  distance_inch: number;
  distance_feet: number;
}

interface UltrasonicVisualizationProps {
  data: UltrasonicData;
  isConnected: boolean;
}

const UltrasonicVisualization: React.FC<UltrasonicVisualizationProps> = ({ data, isConnected }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number>();
  const sweepAngleRef = useRef<number>(0);
  const [sweepAngle, setSweepAngle] = useState(0);

  // HC-SR04 typical specifications
  const MAX_DISTANCE_CM = 400; // Maximum detection range
  const MIN_DISTANCE_CM = 2; // Minimum detection range
  const FIELD_OF_VIEW_DEGREES = 15; // Typical field of view angle

  // Safely get distance values with defaults
  const safeData = {
    distance_cm: data?.distance_cm ?? 0,
    distance_m: data?.distance_m ?? 0,
    distance_inch: data?.distance_inch ?? 0,
    distance_feet: data?.distance_feet ?? 0,
  };

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size based on container
    const updateCanvasSize = () => {
      const container = canvas.parentElement;
      if (container) {
        const rect = container.getBoundingClientRect();
        const size = Math.min(600, rect.width - 32, rect.height - 32);
        canvas.width = size;
        canvas.height = size;
      }
    };

    // Define draw function
    const draw = (timestamp: number) => {
      // Clear canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      const size = canvas.width;
      const centerX = size / 2;
      const centerY = size / 2;
      const radius = Math.min(centerX, centerY) - 40;

      // Draw dark green radar background
      const bgGradient = ctx.createRadialGradient(centerX, centerY, 0, centerX, centerY, radius);
      bgGradient.addColorStop(0, 'rgba(0, 20, 0, 0.8)');
      bgGradient.addColorStop(0.5, 'rgba(0, 30, 0, 0.6)');
      bgGradient.addColorStop(1, 'rgba(0, 40, 0, 0.4)');
      ctx.fillStyle = bgGradient;
      ctx.beginPath();
      ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
      ctx.fill();

      // Draw concentric distance rings
      ctx.strokeStyle = 'rgba(0, 255, 0, 0.3)';
      ctx.lineWidth = 1;
      const numRings = 5;
      for (let i = 1; i <= numRings; i++) {
        const ringRadius = (radius * i) / numRings;
        ctx.beginPath();
        ctx.arc(centerX, centerY, ringRadius, 0, Math.PI * 2);
        ctx.stroke();
      }

      // Draw radial lines (like clock face)
      ctx.strokeStyle = 'rgba(0, 255, 0, 0.2)';
      ctx.lineWidth = 0.5;
      for (let i = 0; i < 12; i++) {
        const angle = (i * Math.PI * 2) / 12 - Math.PI / 2; // Start from top
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(
          centerX + Math.cos(angle) * radius,
          centerY + Math.sin(angle) * radius
        );
        ctx.stroke();
      }

      // Draw distance labels
      ctx.fillStyle = 'rgba(0, 255, 0, 0.7)';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      for (let i = 1; i <= numRings; i++) {
        const distance = (MAX_DISTANCE_CM * i) / numRings;
        const labelRadius = (radius * i) / numRings;
        ctx.fillText(
          `${distance.toFixed(0)}cm`,
          centerX + labelRadius * 0.7,
          centerY - labelRadius * 0.7 + 3
        );
      }

      if (!isConnected || safeData.distance_cm === 0) {
        // Draw disconnected state
        ctx.fillStyle = 'rgba(255, 0, 0, 0.5)';
        ctx.font = 'bold 16px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('NO SIGNAL', centerX, centerY);
        animationFrameRef.current = requestAnimationFrame(draw);
        return;
      }

      // Update sweep angle (rotating beam)
      sweepAngleRef.current += 0.02; // Rotation speed
      if (sweepAngleRef.current > Math.PI * 2) {
        sweepAngleRef.current = 0;
      }
      setSweepAngle(sweepAngleRef.current);

      // Draw sweep trail (fading effect)
      const sweepAngle = sweepAngleRef.current;
      const fovRadians = (FIELD_OF_VIEW_DEGREES * Math.PI) / 180;
      
      // Create sweep gradient
      const sweepGradient = ctx.createRadialGradient(centerX, centerY, 0, centerX, centerY, radius);
      sweepGradient.addColorStop(0, 'rgba(0, 255, 0, 0.1)');
      sweepGradient.addColorStop(0.5, 'rgba(0, 255, 0, 0.05)');
      sweepGradient.addColorStop(1, 'rgba(0, 255, 0, 0)');

      // Draw sweep sector
      ctx.fillStyle = sweepGradient;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.arc(
        centerX,
        centerY,
        radius,
        sweepAngle - fovRadians / 2,
        sweepAngle + fovRadians / 2
      );
      ctx.closePath();
      ctx.fill();

      // Draw active sweep line
      ctx.strokeStyle = 'rgba(0, 255, 0, 0.8)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(
        centerX + Math.cos(sweepAngle) * radius,
        centerY + Math.sin(sweepAngle) * radius
      );
      ctx.stroke();

      // Draw target blip if obstacle detected
      const obstacleDistance = Math.min(safeData.distance_cm, MAX_DISTANCE_CM);
      if (obstacleDistance >= MIN_DISTANCE_CM && obstacleDistance <= MAX_DISTANCE_CM) {
        const blipRadius = (obstacleDistance / MAX_DISTANCE_CM) * radius;
        const blipX = centerX + Math.cos(sweepAngle) * blipRadius;
        const blipY = centerY + Math.sin(sweepAngle) * blipRadius;

        // Draw blip with pulsing effect
        const pulse = Math.sin(Date.now() / 200) * 0.3 + 0.7;
        const blipSize = safeData.distance_cm < 30 ? 8 : safeData.distance_cm < 100 ? 6 : 4;
        
        // Outer glow
        const glowGradient = ctx.createRadialGradient(blipX, blipY, 0, blipX, blipY, blipSize * 2);
        const color = safeData.distance_cm < 30 ? '255, 0, 0' : safeData.distance_cm < 100 ? '255, 200, 0' : '0, 255, 0';
        glowGradient.addColorStop(0, `rgba(${color}, ${0.6 * pulse})`);
        glowGradient.addColorStop(0.5, `rgba(${color}, ${0.3 * pulse})`);
        glowGradient.addColorStop(1, `rgba(${color}, 0)`);
        
        ctx.fillStyle = glowGradient;
        ctx.beginPath();
        ctx.arc(blipX, blipY, blipSize * 2, 0, Math.PI * 2);
        ctx.fill();

        // Inner blip
        ctx.fillStyle = `rgba(${color}, ${pulse})`;
        ctx.beginPath();
        ctx.arc(blipX, blipY, blipSize, 0, Math.PI * 2);
        ctx.fill();

        // Draw line from center to blip
        ctx.strokeStyle = `rgba(${color}, 0.4)`;
        ctx.lineWidth = 1;
        ctx.setLineDash([2, 2]);
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(blipX, blipY);
        ctx.stroke();
        ctx.setLineDash([]);

        // Draw distance label near blip
        ctx.fillStyle = `rgba(${color}, 0.9)`;
        ctx.font = 'bold 11px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(
          `${safeData.distance_cm.toFixed(0)}cm`,
          blipX,
          blipY - blipSize - 8
        );
      }

      // Draw center point (sensor)
      ctx.fillStyle = '#00ff00';
      ctx.beginPath();
      ctx.arc(centerX, centerY, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#00ff00';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Draw crosshair at center
      ctx.strokeStyle = 'rgba(0, 255, 0, 0.5)';
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(centerX - 10, centerY);
      ctx.lineTo(centerX + 10, centerY);
      ctx.moveTo(centerX, centerY - 10);
      ctx.lineTo(centerX, centerY + 10);
      ctx.stroke();

      // Draw warning if too close
      if (safeData.distance_cm < 30 && safeData.distance_cm > 0) {
        ctx.fillStyle = '#ff0000';
        ctx.font = 'bold 14px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('⚠ TOO CLOSE', centerX, 30);
      }

      animationFrameRef.current = requestAnimationFrame(draw);
    };

    // Set up canvas size and resize observer
    updateCanvasSize();
    const resizeObserver = new ResizeObserver(() => {
      updateCanvasSize();
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      animationFrameRef.current = requestAnimationFrame(draw);
    });
    if (canvas.parentElement) {
      resizeObserver.observe(canvas.parentElement);
    }

    draw(0);

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      resizeObserver.disconnect();
    };
  }, [safeData.distance_cm, safeData.distance_m, safeData.distance_inch, safeData.distance_feet, isConnected]);

  // Determine status color
  const getStatusColor = () => {
    if (!isConnected) return 'text-gray-500';
    if (safeData.distance_cm === 0) return 'text-gray-500';
    if (safeData.distance_cm < 30) return 'text-red-500';
    if (safeData.distance_cm < 100) return 'text-yellow-500';
    return 'text-green-500';
  };

  return (
    <div className="w-full h-full flex flex-col items-center justify-center p-4 bg-gray-900 rounded-lg">
      <div className="w-full max-w-2xl">
        {/* Header with status */}
        <div className="flex items-center justify-between mb-4">
          <div className="flex items-center gap-2">
            <Radar className="h-5 w-5 text-green-400 animate-spin" style={{ animationDuration: '3s' }} />
            <h3 className="text-lg font-semibold text-white">Radar Display</h3>
          </div>
          <div className={`flex items-center gap-2 ${getStatusColor()}`}>
            {safeData.distance_cm < 30 && safeData.distance_cm > 0 && (
              <AlertTriangle className="h-4 w-4" />
            )}
            <span className="text-sm font-mono">
              {isConnected ? `${safeData.distance_cm.toFixed(1)} cm` : 'Disconnected'}
            </span>
          </div>
        </div>

        {/* Canvas visualization */}
        <div className="relative bg-black rounded-lg border-2 border-green-500 p-4 flex items-center justify-center shadow-lg shadow-green-500/20">
          <canvas
            ref={canvasRef}
            className="w-full h-auto rounded"
            style={{ maxWidth: '600px', maxHeight: '600px', aspectRatio: '1' }}
          />
        </div>

        {/* Distance info panel */}
        <div className="mt-4 grid grid-cols-2 sm:grid-cols-4 gap-4">
          <div className="bg-gray-800 p-3 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-1">Centimeters</div>
            <div className="text-lg font-mono font-bold text-green-400">
              {safeData.distance_cm.toFixed(1)}
            </div>
          </div>
          <div className="bg-gray-800 p-3 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-1">Meters</div>
            <div className="text-lg font-mono font-bold text-green-400">
              {safeData.distance_m.toFixed(2)}
            </div>
          </div>
          <div className="bg-gray-800 p-3 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-1">Inches</div>
            <div className="text-lg font-mono font-bold text-green-400">
              {safeData.distance_inch.toFixed(2)}
            </div>
          </div>
          <div className="bg-gray-800 p-3 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-1">Feet</div>
            <div className="text-lg font-mono font-bold text-green-400">
              {safeData.distance_feet.toFixed(2)}
            </div>
          </div>
        </div>

        {/* Status info */}
        <div className="mt-4 text-xs text-gray-400 text-center">
          {isConnected ? (
            <>
              <span className="text-green-400">●</span> Active • Range: {MIN_DISTANCE_CM}-{MAX_DISTANCE_CM} cm • FOV: {FIELD_OF_VIEW_DEGREES}°
            </>
          ) : (
            <span className="text-red-400">●</span> Waiting for sensor connection...
          )}
        </div>
      </div>
    </div>
  );
};

export default UltrasonicVisualization;
