/**
 * RobotController Component
 * 
 * Joystick/button interface for controlling robot via ROS2 cmd_vel.
 * 
 * Usage:
 *   <RobotController />
 */

import React, { useEffect, useState, useRef } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { buildGatewayUrl } from '@/utils/gateway';
import { robotWS } from '@/utils/ws';
import { useRobotOnline } from '@/hooks/useRobotOnline';

export function RobotController() {
  const robotOnline = useRobotOnline();
  const [connected, setConnected] = useState(false);
  const [linearSpeed, setLinearSpeed] = useState(0);
  const [angularSpeed, setAngularSpeed] = useState(0);
  const wsRef = useRef<WebSocket | null>(null);
  const commandIntervalRef = useRef<NodeJS.Timeout | null>(null);

  useEffect(() => {
    if (!robotOnline) {
      setConnected(false);
      return;
    }

    let mounted = true;

    const connect = async () => {
      try {
        const wsUrl = await buildGatewayUrl('/ws/ros/bridge');
        const protocol = wsUrl.startsWith('https') ? 'wss' : 'ws';
        const url = wsUrl.replace(/^https?/, protocol);

        const ws = robotWS(url);
        if (!ws) {
          setConnected(false);
          return;
        }
        wsRef.current = ws;

        ws.onopen = () => {
          if (!mounted) return;
          setConnected(true);
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
      if (commandIntervalRef.current) {
        clearInterval(commandIntervalRef.current);
      }
      // Send stop command
      if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
        wsRef.current.send(JSON.stringify({
          type: 'publish',
          topic: '/cmd_vel',
          msg_type: 'Twist',
          command: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }
        }));
        wsRef.current.close();
      }
    };
  }, [robotOnline]);

  const sendCommand = (linear: number, angular: number) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      return;
    }

    wsRef.current.send(JSON.stringify({
      type: 'publish',
      topic: '/cmd_vel',
      msg_type: 'Twist',
      command: {
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular }
      }
    }));

    setLinearSpeed(linear);
    setAngularSpeed(angular);
  };

  const stop = () => {
    sendCommand(0, 0);
    if (commandIntervalRef.current) {
      clearInterval(commandIntervalRef.current);
      commandIntervalRef.current = null;
    }
  };

  const startContinuousCommand = (linear: number, angular: number) => {
    stop(); // Clear any existing interval
    sendCommand(linear, angular);
    
    // Send command repeatedly while button is held
    commandIntervalRef.current = setInterval(() => {
      sendCommand(linear, angular);
    }, 100); // 10 Hz
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle>Robot Control</CardTitle>
        <div className="text-xs text-muted-foreground">
          Status: {connected ? 'Connected' : 'Disconnected'}
        </div>
      </CardHeader>
      <CardContent>
        <div className="grid grid-cols-3 gap-2 mb-4">
          <div></div>
          <Button
            variant="outline"
            onMouseDown={() => startContinuousCommand(0.5, 0)}
            onMouseUp={stop}
            onMouseLeave={stop}
            disabled={!connected}
          >
            ↑ Forward
          </Button>
          <div></div>

          <Button
            variant="outline"
            onMouseDown={() => startContinuousCommand(0, 0.5)}
            onMouseUp={stop}
            onMouseLeave={stop}
            disabled={!connected}
          >
            ← Left
          </Button>
          <Button
            variant="destructive"
            onClick={stop}
            disabled={!connected}
          >
            Stop
          </Button>
          <Button
            variant="outline"
            onMouseDown={() => startContinuousCommand(0, -0.5)}
            onMouseUp={stop}
            onMouseLeave={stop}
            disabled={!connected}
          >
            Right →
          </Button>

          <div></div>
          <Button
            variant="outline"
            onMouseDown={() => startContinuousCommand(-0.5, 0)}
            onMouseUp={stop}
            onMouseLeave={stop}
            disabled={!connected}
          >
            ↓ Backward
          </Button>
          <div></div>
        </div>

        <div className="text-xs text-muted-foreground">
          Linear: {linearSpeed.toFixed(2)} | Angular: {angularSpeed.toFixed(2)}
        </div>
      </CardContent>
    </Card>
  );
}

