/*
# File: /src/components/sensors/UltrasonicVisualization.tsx
# Summary:
#   Forward-facing radar display of ultrasonic sensor data showing:
#   - Forward-facing arc (sensor pointing ahead)
#   - Green for free space, red for obstacles
#   - Rotates only when robot rotates (not continuously)
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
  robotHeading?: number; // Optional: robot's heading in radians (0 = forward/up)
}

const UltrasonicVisualization: React.FC<UltrasonicVisualizationProps> = ({ 
  data, 
  isConnected,
  robotHeading = -Math.PI / 2 // Default: pointing forward (up)
}) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const animationFrameRef = useRef<number>();

  // HC-SR04 typical specifications
  const MAX_DISTANCE_CM = 400; // Maximum detection range
  const MIN_DISTANCE_CM = 2; // Minimum detection range
  const FIELD_OF_VIEW_DEGREES = 15; // Typical field of view angle
  const FORWARD_ARC_DEGREES = 180; // Forward-facing arc to display (wider view)

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
    const draw = () => {
      // Clear canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      const size = canvas.width;
      const centerX = size / 2;
      const centerY = size / 2;
      const radius = Math.min(centerX, centerY) - 40;

      // Draw dark background
      const bgGradient = ctx.createRadialGradient(centerX, centerY, 0, centerX, centerY, radius);
      bgGradient.addColorStop(0, 'rgba(0, 0, 0, 0.9)');
      bgGradient.addColorStop(0.5, 'rgba(0, 0, 0, 0.7)');
      bgGradient.addColorStop(1, 'rgba(0, 0, 0, 0.5)');
      ctx.fillStyle = bgGradient;
      ctx.beginPath();
      ctx.arc(centerX, centerY, radius, 0, Math.PI * 2);
      ctx.fill();

      // Draw concentric distance rings (only in forward arc)
      ctx.strokeStyle = 'rgba(100, 100, 100, 0.3)';
      ctx.lineWidth = 1;
      const numRings = 5;
      const arcStart = robotHeading - (FORWARD_ARC_DEGREES * Math.PI) / 360;
      const arcEnd = robotHeading + (FORWARD_ARC_DEGREES * Math.PI) / 360;
      
      for (let i = 1; i <= numRings; i++) {
        const ringRadius = (radius * i) / numRings;
        ctx.beginPath();
        ctx.arc(centerX, centerY, ringRadius, arcStart, arcEnd);
        ctx.stroke();
      }

      // Draw radial lines within forward arc
      ctx.strokeStyle = 'rgba(100, 100, 100, 0.2)';
      ctx.lineWidth = 0.5;
      const numLines = 8;
      for (let i = 0; i <= numLines; i++) {
        const angle = arcStart + ((arcEnd - arcStart) * i) / numLines;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(
          centerX + Math.cos(angle) * radius,
          centerY + Math.sin(angle) * radius
        );
        ctx.stroke();
      }

      // Draw distance labels
      ctx.fillStyle = 'rgba(200, 200, 200, 0.7)';
      ctx.font = '10px monospace';
      ctx.textAlign = 'center';
      for (let i = 1; i <= numRings; i++) {
        const distance = (MAX_DISTANCE_CM * i) / numRings;
        const labelRadius = (radius * i) / numRings;
        const labelAngle = robotHeading;
        ctx.fillText(
          `${distance.toFixed(0)}cm`,
          centerX + Math.cos(labelAngle) * labelRadius * 0.8,
          centerY + Math.sin(labelAngle) * labelRadius * 0.8 + 3
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

      // Determine if obstacle detected
      const obstacleDistance = Math.min(safeData.distance_cm, MAX_DISTANCE_CM);
      const hasObstacle = obstacleDistance >= MIN_DISTANCE_CM && obstacleDistance <= MAX_DISTANCE_CM;
      
      // Color coding: Green for free space (far), Red for obstacles (close)
      const isObstacle = safeData.distance_cm < 100; // Obstacle threshold: < 100cm
      const isCloseObstacle = safeData.distance_cm < 30; // Close obstacle: < 30cm
      
      // Draw forward-facing detection arc
      const fovRadians = (FIELD_OF_VIEW_DEGREES * Math.PI) / 180;
      const detectionRadius = hasObstacle ? (obstacleDistance / MAX_DISTANCE_CM) * radius : radius;
      
      // Create gradient based on obstacle status
      let arcColor: string;
      if (isCloseObstacle) {
        arcColor = 'rgba(255, 0, 0, 0.6)'; // Red for close obstacles
      } else if (isObstacle) {
        arcColor = 'rgba(255, 100, 0, 0.5)'; // Orange for medium obstacles
      } else {
        arcColor = 'rgba(0, 255, 0, 0.4)'; // Green for free space
      }

      // Draw detection arc (forward-facing)
      ctx.fillStyle = arcColor;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.arc(
        centerX,
        centerY,
        detectionRadius,
        robotHeading - fovRadians / 2,
        robotHeading + fovRadians / 2
      );
      ctx.closePath();
      ctx.fill();

      // Draw free space arc (green) beyond obstacle if obstacle detected
      if (hasObstacle && obstacleDistance < MAX_DISTANCE_CM) {
        const freeSpaceStartRadius = detectionRadius;
        const freeSpaceEndRadius = radius;
        
        const freeSpaceGradient = ctx.createRadialGradient(
          centerX + Math.cos(robotHeading) * freeSpaceStartRadius,
          centerY + Math.sin(robotHeading) * freeSpaceStartRadius,
          0,
          centerX + Math.cos(robotHeading) * freeSpaceEndRadius,
          centerY + Math.sin(robotHeading) * freeSpaceEndRadius,
          freeSpaceEndRadius - freeSpaceStartRadius
        );
        freeSpaceGradient.addColorStop(0, 'rgba(0, 255, 0, 0.2)');
        freeSpaceGradient.addColorStop(1, 'rgba(0, 255, 0, 0.05)');
        
        ctx.fillStyle = freeSpaceGradient;
        ctx.beginPath();
        ctx.moveTo(
          centerX + Math.cos(robotHeading - fovRadians / 2) * freeSpaceStartRadius,
          centerY + Math.sin(robotHeading - fovRadians / 2) * freeSpaceStartRadius
        );
        ctx.lineTo(
          centerX + Math.cos(robotHeading - fovRadians / 2) * freeSpaceEndRadius,
          centerY + Math.sin(robotHeading - fovRadians / 2) * freeSpaceEndRadius
        );
        ctx.arc(
          centerX,
          centerY,
          freeSpaceEndRadius,
          robotHeading - fovRadians / 2,
          robotHeading + fovRadians / 2
        );
        ctx.lineTo(
          centerX + Math.cos(robotHeading + fovRadians / 2) * freeSpaceStartRadius,
          centerY + Math.sin(robotHeading + fovRadians / 2) * freeSpaceStartRadius
        );
        ctx.closePath();
        ctx.fill();
      } else if (!hasObstacle) {
        // No obstacle detected - show all green (free space)
        const freeSpaceGradient = ctx.createRadialGradient(centerX, centerY, 0, centerX, centerY, radius);
        freeSpaceGradient.addColorStop(0, 'rgba(0, 255, 0, 0.3)');
        freeSpaceGradient.addColorStop(0.5, 'rgba(0, 255, 0, 0.2)');
        freeSpaceGradient.addColorStop(1, 'rgba(0, 255, 0, 0.05)');
        
        ctx.fillStyle = freeSpaceGradient;
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.arc(
          centerX,
          centerY,
          radius,
          robotHeading - fovRadians / 2,
          robotHeading + fovRadians / 2
        );
        ctx.closePath();
        ctx.fill();
      }

      // Draw center line (forward direction)
      ctx.strokeStyle = isObstacle ? 'rgba(255, 0, 0, 0.8)' : 'rgba(0, 255, 0, 0.8)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(
        centerX + Math.cos(robotHeading) * detectionRadius,
        centerY + Math.sin(robotHeading) * detectionRadius
      );
      ctx.stroke();

      // Draw obstacle indicator if detected
      if (hasObstacle) {
        const obstacleX = centerX + Math.cos(robotHeading) * detectionRadius;
        const obstacleY = centerY + Math.sin(robotHeading) * detectionRadius;
        
        // Draw obstacle blip
        const pulse = Math.sin(Date.now() / 200) * 0.3 + 0.7;
        const blipSize = isCloseObstacle ? 10 : 8;
        const blipColor = isCloseObstacle ? '255, 0, 0' : '255, 100, 0';
        
        // Outer glow
        const glowGradient = ctx.createRadialGradient(obstacleX, obstacleY, 0, obstacleX, obstacleY, blipSize * 2);
        glowGradient.addColorStop(0, `rgba(${blipColor}, ${0.8 * pulse})`);
        glowGradient.addColorStop(0.5, `rgba(${blipColor}, ${0.4 * pulse})`);
        glowGradient.addColorStop(1, `rgba(${blipColor}, 0)`);
        
        ctx.fillStyle = glowGradient;
        ctx.beginPath();
        ctx.arc(obstacleX, obstacleY, blipSize * 2, 0, Math.PI * 2);
        ctx.fill();

        // Inner blip
        ctx.fillStyle = `rgba(${blipColor}, ${pulse})`;
        ctx.beginPath();
        ctx.arc(obstacleX, obstacleY, blipSize, 0, Math.PI * 2);
        ctx.fill();

        // Draw distance label
        ctx.fillStyle = `rgba(${blipColor}, 0.9)`;
        ctx.font = 'bold 12px monospace';
        ctx.textAlign = 'center';
        ctx.fillText(
          `${safeData.distance_cm.toFixed(0)}cm`,
          obstacleX,
          obstacleY - blipSize - 10
        );
      }

      // Draw sensor (at center)
      ctx.fillStyle = isObstacle ? '#ff0000' : '#00ff00';
      ctx.beginPath();
      ctx.arc(centerX, centerY, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Draw forward indicator arrow
      ctx.strokeStyle = isObstacle ? 'rgba(255, 0, 0, 0.6)' : 'rgba(0, 255, 0, 0.6)';
      ctx.lineWidth = 2;
      ctx.beginPath();
      const arrowLength = 20;
      const arrowX = centerX + Math.cos(robotHeading) * arrowLength;
      const arrowY = centerY + Math.sin(robotHeading) * arrowLength;
      ctx.moveTo(centerX, centerY);
      ctx.lineTo(arrowX, arrowY);
      // Arrow head
      ctx.lineTo(
        arrowX - Math.cos(robotHeading + Math.PI * 0.8) * 8,
        arrowY - Math.sin(robotHeading + Math.PI * 0.8) * 8
      );
      ctx.moveTo(arrowX, arrowY);
      ctx.lineTo(
        arrowX - Math.cos(robotHeading - Math.PI * 0.8) * 8,
        arrowY - Math.sin(robotHeading - Math.PI * 0.8) * 8
      );
      ctx.stroke();

      // Draw warning if too close
      if (isCloseObstacle) {
        ctx.fillStyle = '#ff0000';
        ctx.font = 'bold 16px monospace';
        ctx.textAlign = 'center';
        ctx.fillText('⚠ OBSTACLE DETECTED', centerX, 30);
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

    draw();

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      resizeObserver.disconnect();
    };
  }, [safeData.distance_cm, safeData.distance_m, safeData.distance_inch, safeData.distance_feet, isConnected, robotHeading]);

  // Determine status color
  const getStatusColor = () => {
    if (!isConnected) return 'text-gray-500';
    if (safeData.distance_cm === 0) return 'text-gray-500';
    if (safeData.distance_cm < 30) return 'text-red-500';
    if (safeData.distance_cm < 100) return 'text-yellow-500';
    return 'text-green-500';
  };

  return (
    <div className="w-full h-full flex flex-col items-center justify-center p-2 bg-gray-900 rounded-lg">
      <div className="w-full">
        {/* Header with status */}
        <div className="flex items-center justify-between mb-2">
          <div className="flex items-center gap-2">
            <Radar className="h-4 w-4 text-green-400" />
            <h3 className="text-base font-semibold text-white">Forward Sensor Display</h3>
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
        <div className="relative bg-black rounded-lg border-2 border-green-500 p-2 flex items-center justify-center shadow-lg shadow-green-500/20">
          <canvas
            ref={canvasRef}
            className="w-full h-auto rounded"
            style={{ maxWidth: '500px', maxHeight: '500px', aspectRatio: '1' }}
          />
        </div>

        {/* Distance info panel */}
        <div className="mt-2 grid grid-cols-2 sm:grid-cols-4 gap-2">
          <div className="bg-gray-800 p-2 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-0.5">Centimeters</div>
            <div className="text-base font-mono font-bold text-green-400">
              {safeData.distance_cm.toFixed(1)}
            </div>
          </div>
          <div className="bg-gray-800 p-2 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-0.5">Meters</div>
            <div className="text-base font-mono font-bold text-green-400">
              {safeData.distance_m.toFixed(2)}
            </div>
          </div>
          <div className="bg-gray-800 p-2 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-0.5">Inches</div>
            <div className="text-base font-mono font-bold text-green-400">
              {safeData.distance_inch.toFixed(2)}
            </div>
          </div>
          <div className="bg-gray-800 p-2 rounded-lg text-center border border-green-500/30">
            <div className="text-xs text-gray-400 mb-0.5">Feet</div>
            <div className="text-base font-mono font-bold text-green-400">
              {safeData.distance_feet.toFixed(2)}
            </div>
          </div>
        </div>

        {/* Status info */}
        <div className="mt-2 text-xs text-gray-400 text-center">
          {isConnected ? (
            <>
              <span className="text-green-400 inline-block w-2 h-2 rounded-full bg-green-400 mr-1"></span>
              Active • Range: {MIN_DISTANCE_CM}-{MAX_DISTANCE_CM} cm • FOV: {FIELD_OF_VIEW_DEGREES}° • Forward-facing
            </>
          ) : (
            <>
              <span className="text-red-400 inline-block w-2 h-2 rounded-full bg-red-400 mr-1"></span>
              Waiting for sensor connection...
            </>
          )}
        </div>
      </div>
    </div>
  );
};

export default UltrasonicVisualization;
