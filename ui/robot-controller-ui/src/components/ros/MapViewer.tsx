/**
 * MapViewer Component
 * 
 * Displays robot map and path visualization.
 * Shows robot position, goal, and planned path.
 * 
 * Usage:
 *   <MapViewer />
 */

import React, { useEffect, useState, useRef } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { buildGatewayUrl } from '@/utils/gateway';

interface MapViewerProps {
  width?: number;
  height?: number;
  className?: string;
}

interface RobotPose {
  x: number;
  y: number;
  theta: number;
}

interface PathPoint {
  x: number;
  y: number;
}

export function MapViewer({
  width = 600,
  height = 600,
  className = ''
}: MapViewerProps) {
  const [connected, setConnected] = useState(false);
  const [robotPose, setRobotPose] = useState<RobotPose | null>(null);
  const [path, setPath] = useState<PathPoint[]>([]);
  const [goal, setGoal] = useState<PathPoint | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wsRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    let mounted = true;

    const connect = async () => {
      try {
        const wsUrl = await buildGatewayUrl('/ws/ros/bridge');
        const protocol = wsUrl.startsWith('https') ? 'wss' : 'ws';
        const url = wsUrl.replace(/^https?/, protocol);

        const ws = new WebSocket(url);
        wsRef.current = ws;

        ws.onopen = () => {
          if (!mounted) return;
          setConnected(true);

          // Subscribe to odometry
          ws.send(JSON.stringify({
            type: "subscribe",
            topic: "/odom",
            msg_type: "Odometry"
          }));

          // Subscribe to path
          ws.send(JSON.stringify({
            type: "subscribe",
            topic: "/plan",
            msg_type: "Path"
          }));
        };

        ws.onmessage = (event) => {
          if (!mounted) return;
          try {
            const msg = JSON.parse(event.data);

            if (msg.type === "ros2_message") {
              if (msg.topic === "/odom") {
                // Update robot pose
                const pose = msg.data.pose?.pose;
                if (pose) {
                  const x = pose.position?.x || 0;
                  const y = pose.position?.y || 0;
                  // Extract yaw from quaternion
                  const q = pose.orientation || {};
                  const yaw = Math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z)
                  );
                  setRobotPose({ x, y, theta: yaw });
                }
              } else if (msg.topic === "/plan") {
                // Update path
                const poses = msg.data.poses || [];
                const pathPoints = poses.map((p: any) => ({
                  x: p.pose?.position?.x || 0,
                  y: p.pose?.position?.y || 0
                }));
                setPath(pathPoints);
              }
            }
          } catch (e) {
            console.error('Error parsing message:', e);
          }
        };

        ws.onerror = () => {
          if (!mounted) return;
          setConnected(false);
        };

        ws.onclose = () => {
          if (!mounted) return;
          setConnected(false);
          wsRef.current = null;
        };
      } catch (err) {
        console.error('Connection error:', err);
        setConnected(false);
      }
    };

    connect();

    return () => {
      mounted = false;
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Clear canvas
    ctx.clearRect(0, 0, width, height);

    // Set up coordinate system (origin at center, y-up)
    ctx.save();
    ctx.translate(width / 2, height / 2);
    ctx.scale(1, -1); // Flip y-axis

    // Draw grid
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 1;
    const gridSize = 50;
    for (let x = -width; x < width; x += gridSize) {
      ctx.beginPath();
      ctx.moveTo(x, -height);
      ctx.lineTo(x, height);
      ctx.stroke();
    }
    for (let y = -height; y < height; y += gridSize) {
      ctx.beginPath();
      ctx.moveTo(-width, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }

    // Draw axes
    ctx.strokeStyle = '#999';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-width, 0);
    ctx.lineTo(width, 0);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, -height);
    ctx.lineTo(0, height);
    ctx.stroke();

    // Draw path
    if (path.length > 1) {
      ctx.strokeStyle = '#3b82f6';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(path[0].x * 100, path[0].y * 100);
      for (let i = 1; i < path.length; i++) {
        ctx.lineTo(path[i].x * 100, path[i].y * 100);
      }
      ctx.stroke();

      // Draw path points
      ctx.fillStyle = '#3b82f6';
      path.forEach((point) => {
        ctx.beginPath();
        ctx.arc(point.x * 100, point.y * 100, 3, 0, 2 * Math.PI);
        ctx.fill();
      });
    }

    // Draw goal
    if (goal) {
      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(goal.x * 100, goal.y * 100, 8, 0, 2 * Math.PI);
      ctx.fill();
    }

    // Draw robot
    if (robotPose) {
      const scale = 100; // pixels per meter
      const x = robotPose.x * scale;
      const y = robotPose.y * scale;
      const theta = robotPose.theta;

      // Robot body (circle)
      ctx.fillStyle = '#10b981';
      ctx.beginPath();
      ctx.arc(x, y, 10, 0, 2 * Math.PI);
      ctx.fill();

      // Robot heading (arrow)
      ctx.strokeStyle = '#10b981';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(x, y);
      ctx.lineTo(
        x + Math.cos(theta) * 15,
        y + Math.sin(theta) * 15
      );
      ctx.stroke();
    }

    ctx.restore();
  }, [robotPose, path, goal, width, height]);

  const handleCanvasClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = (e.clientX - rect.left - width / 2) / 100;
    const y = -(e.clientY - rect.top - height / 2) / 100;

    setGoal({ x, y });

    // Send goal to ROS2
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      // Would send goal pose to /goal_pose topic
      // For now, just update local state
    }
  };

  return (
    <Card className={className}>
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-sm font-medium">Map & Navigation</CardTitle>
          <Badge variant={connected ? 'default' : 'secondary'}>
            {connected ? 'Connected' : 'Disconnected'}
          </Badge>
        </div>
      </CardHeader>
      <CardContent>
        <canvas
          ref={canvasRef}
          width={width}
          height={height}
          onClick={handleCanvasClick}
          className="border rounded cursor-crosshair"
          style={{ maxWidth: '100%' }}
        />
        <div className="text-xs text-muted-foreground mt-2">
          Click on map to set goal | Robot: {robotPose ? `(${robotPose.x.toFixed(2)}, ${robotPose.y.toFixed(2)})` : 'No data'}
        </div>
      </CardContent>
    </Card>
  );
}

