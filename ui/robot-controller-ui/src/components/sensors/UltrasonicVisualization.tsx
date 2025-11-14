/*
# File: /src/components/sensors/UltrasonicVisualization.tsx
# Summary:
#   Visual display of ultrasonic sensor data showing:
#   - Sensor's line of sight (cone/beam visualization)
#   - Detected distance with markers
#   - Obstacles/barriers at detected distance
#   - Real-time distance readings
*/

'use client';

import React, { useEffect, useRef } from 'react';
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

  // HC-SR04 typical specifications
  const MAX_DISTANCE_CM = 400; // Maximum detection range
  const MIN_DISTANCE_CM = 2; // Minimum detection range
  const FIELD_OF_VIEW_DEGREES = 15; // Typical field of view angle

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
        canvas.width = Math.min(600, rect.width - 32); // Account for padding
        canvas.height = 500;
      }
    };

    // Define draw function first
    const draw = () => {
      // Clear canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      // Set canvas size
      const width = canvas.width;
      const height = canvas.height;
      const centerX = width / 2;
      const sensorY = height - 40; // Sensor position at bottom
      const sensorRadius = 20;

      // Calculate visualization scale
      const maxDisplayDistance = Math.max(MAX_DISTANCE_CM, data.distance_cm || MAX_DISTANCE_CM);
      const scale = (height - 100) / maxDisplayDistance; // Leave space for sensor and labels

      // Draw background grid
      ctx.strokeStyle = 'rgba(100, 100, 100, 0.3)';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 5; i++) {
        const y = sensorY - (i * maxDisplayDistance * scale) / 5;
        ctx.beginPath();
        ctx.moveTo(20, y);
        ctx.lineTo(width - 20, y);
        ctx.stroke();
      }

      // Draw distance markers on the right
      ctx.fillStyle = 'rgba(200, 200, 200, 0.6)';
      ctx.font = '12px monospace';
      ctx.textAlign = 'right';
      for (let i = 0; i <= 5; i++) {
        const distance = (i * maxDisplayDistance) / 5;
        const y = sensorY - (distance * scale);
        ctx.fillText(`${distance.toFixed(0)} cm`, width - 25, y + 4);
      }

      if (!isConnected || data.distance_cm === 0) {
        // Draw disconnected state
        ctx.fillStyle = 'rgba(100, 100, 100, 0.5)';
        ctx.font = '16px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('Sensor Disconnected', centerX, height / 2);
        animationFrameRef.current = requestAnimationFrame(draw);
        return;
      }

      // Calculate obstacle position
      const obstacleDistance = Math.min(data.distance_cm, MAX_DISTANCE_CM);
      const obstacleY = sensorY - (obstacleDistance * scale);

      // Draw sensor (at bottom center)
      ctx.fillStyle = '#3b82f6';
      ctx.beginPath();
      ctx.arc(centerX, sensorY, sensorRadius, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#1e40af';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Draw sensor label
      ctx.fillStyle = '#ffffff';
      ctx.font = '12px sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText('HC-SR04', centerX, sensorY + sensorRadius + 15);

      // Draw field of view cone
      const fovRadians = (FIELD_OF_VIEW_DEGREES * Math.PI) / 180;
      const coneLength = obstacleDistance * scale;

      // Create gradient for the cone
      const gradient = ctx.createLinearGradient(centerX, sensorY, centerX, obstacleY);
      gradient.addColorStop(0, 'rgba(59, 130, 246, 0.3)'); // Blue at sensor
      gradient.addColorStop(0.5, 'rgba(59, 130, 246, 0.2)');
      gradient.addColorStop(1, 'rgba(59, 130, 246, 0.05)'); // Fade at end

      ctx.fillStyle = gradient;
      ctx.beginPath();
      ctx.moveTo(centerX, sensorY);
      const leftAngle = -fovRadians / 2;
      const rightAngle = fovRadians / 2;
      ctx.lineTo(
        centerX + Math.sin(leftAngle) * coneLength,
        sensorY - Math.cos(leftAngle) * coneLength
      );
      ctx.lineTo(
        centerX + Math.sin(rightAngle) * coneLength,
        sensorY - Math.cos(rightAngle) * coneLength
      );
      ctx.closePath();
      ctx.fill();

      // Draw detection beam (line of sight)
      ctx.strokeStyle = '#60a5fa';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(centerX, sensorY);
      ctx.lineTo(centerX, obstacleY);
      ctx.stroke();
      ctx.setLineDash([]);

      // Draw obstacle/barrier
      if (obstacleDistance <= MAX_DISTANCE_CM && obstacleDistance >= MIN_DISTANCE_CM) {
        const obstacleWidth = 60;
        const obstacleHeight = 20;

        // Draw obstacle shadow
        ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
        ctx.fillRect(
          centerX - obstacleWidth / 2 + 3,
          obstacleY - obstacleHeight / 2 + 3,
          obstacleWidth,
          obstacleHeight
        );

        // Draw obstacle
        ctx.fillStyle = data.distance_cm < 30 ? '#ef4444' : data.distance_cm < 100 ? '#f59e0b' : '#10b981';
        ctx.fillRect(
          centerX - obstacleWidth / 2,
          obstacleY - obstacleHeight / 2,
          obstacleWidth,
          obstacleHeight
        );

        // Draw obstacle border
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 2;
        ctx.strokeRect(
          centerX - obstacleWidth / 2,
          obstacleY - obstacleHeight / 2,
          obstacleWidth,
          obstacleHeight
        );

        // Draw distance line from sensor to obstacle
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(centerX, sensorY);
        ctx.lineTo(centerX, obstacleY);
        ctx.stroke();
        ctx.setLineDash([]);
      }

      // Draw distance label at obstacle
      if (obstacleDistance <= MAX_DISTANCE_CM && obstacleDistance >= MIN_DISTANCE_CM) {
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 14px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText(
          `${data.distance_cm.toFixed(1)} cm`,
          centerX,
          obstacleY - obstacleHeight / 2 - 10
        );
      }

      // Draw warning if too close
      if (data.distance_cm < 30 && data.distance_cm > 0) {
        ctx.fillStyle = '#ef4444';
        ctx.font = 'bold 16px sans-serif';
        ctx.textAlign = 'center';
        ctx.fillText('⚠ TOO CLOSE', centerX, 30);
      }

      animationFrameRef.current = requestAnimationFrame(draw);
    };

    // Set up canvas size and resize observer
    updateCanvasSize();
    const resizeObserver = new ResizeObserver(() => {
      updateCanvasSize();
      // Redraw after resize
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      animationFrameRef.current = requestAnimationFrame(draw);
    });
    if (canvas.parentElement) {
      resizeObserver.observe(canvas.parentElement);
    }

    draw();

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      resizeObserver.disconnect();
    };
  }, [data, isConnected]);

  // Determine status color
  const getStatusColor = () => {
    if (!isConnected) return 'text-gray-500';
    if (data.distance_cm === 0) return 'text-gray-500';
    if (data.distance_cm < 30) return 'text-red-500';
    if (data.distance_cm < 100) return 'text-yellow-500';
    return 'text-green-500';
  };

  return (
    <div className="w-full h-full flex flex-col items-center justify-center p-4 bg-gray-900 rounded-lg">
      <div className="w-full max-w-2xl">
        {/* Header with status */}
        <div className="flex items-center justify-between mb-4">
          <div className="flex items-center gap-2">
            <Radar className="h-5 w-5 text-blue-400" />
            <h3 className="text-lg font-semibold text-white">Ultrasonic Sensor Visualization</h3>
          </div>
          <div className={`flex items-center gap-2 ${getStatusColor()}`}>
            {data.distance_cm < 30 && data.distance_cm > 0 && (
              <AlertTriangle className="h-4 w-4" />
            )}
            <span className="text-sm font-mono">
              {isConnected ? `${data.distance_cm.toFixed(1)} cm` : 'Disconnected'}
            </span>
          </div>
        </div>

        {/* Canvas visualization */}
        <div className="relative bg-gray-950 rounded-lg border border-gray-700 p-4 flex items-center justify-center">
          <canvas
            ref={canvasRef}
            className="w-full max-w-[600px] h-auto"
            style={{ maxHeight: '500px' }}
          />
        </div>

        {/* Distance info panel */}
        <div className="mt-4 grid grid-cols-2 sm:grid-cols-4 gap-4">
          <div className="bg-gray-800 p-3 rounded-lg text-center">
            <div className="text-xs text-gray-400 mb-1">Centimeters</div>
            <div className="text-lg font-mono font-bold text-white">
              {data.distance_cm.toFixed(1)}
            </div>
          </div>
          <div className="bg-gray-800 p-3 rounded-lg text-center">
            <div className="text-xs text-gray-400 mb-1">Meters</div>
            <div className="text-lg font-mono font-bold text-white">
              {data.distance_m.toFixed(2)}
            </div>
          </div>
          <div className="bg-gray-800 p-3 rounded-lg text-center">
            <div className="text-xs text-gray-400 mb-1">Inches</div>
            <div className="text-lg font-mono font-bold text-white">
              {data.distance_inch.toFixed(2)}
            </div>
          </div>
          <div className="bg-gray-800 p-3 rounded-lg text-center">
            <div className="text-xs text-gray-400 mb-1">Feet</div>
            <div className="text-lg font-mono font-bold text-white">
              {data.distance_feet.toFixed(2)}
            </div>
          </div>
        </div>

        {/* Status info */}
        <div className="mt-4 text-xs text-gray-400 text-center">
          {isConnected ? (
            <>
              Field of View: {FIELD_OF_VIEW_DEGREES}° • Range: {MIN_DISTANCE_CM}-{MAX_DISTANCE_CM} cm
            </>
          ) : (
            'Waiting for sensor connection...'
          )}
        </div>
      </div>
    </div>
  );
};

export default UltrasonicVisualization;

