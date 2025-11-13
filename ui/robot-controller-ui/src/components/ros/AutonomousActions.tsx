/**
 * AutonomousActions Component
 * 
 * Control panel for triggering autonomous behaviors via ROS2 actions.
 * 
 * Usage:
 *   <AutonomousActions />
 */

import React, { useEffect, useState, useRef } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { buildGatewayUrl } from '@/utils/gateway';

interface ActionStatus {
  action: string;
  goal_id: string;
  status: 'active' | 'completed' | 'cancelled' | 'error';
  feedback?: any;
  result?: any;
}

export function AutonomousActions() {
  const [connected, setConnected] = useState(false);
  const [activeActions, setActiveActions] = useState<Map<string, ActionStatus>>(new Map());
  const wsRef = useRef<WebSocket | null>(null);
  
  // Goal inputs
  const [navigateX, setNavigateX] = useState('1.0');
  const [navigateY, setNavigateY] = useState('0.0');
  const [followLineDuration, setFollowLineDuration] = useState('10');
  const [obstacleDirection, setObstacleDirection] = useState('forward');

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
        };

        ws.onmessage = (event) => {
          if (!mounted) return;
          try {
            const msg = JSON.parse(event.data);

            if (msg.type === 'action_goal_accepted') {
              setActiveActions(prev => {
                const newMap = new Map(prev);
                newMap.set(msg.goal_id, {
                  action: msg.action,
                  goal_id: msg.goal_id,
                  status: 'active'
                });
                return newMap;
              });
            } else if (msg.type === 'action_feedback') {
              setActiveActions(prev => {
                const newMap = new Map(prev);
                const existing = newMap.get(msg.goal_id);
                if (existing) {
                  newMap.set(msg.goal_id, {
                    ...existing,
                    feedback: msg.feedback
                  });
                }
                return newMap;
              });
            } else if (msg.type === 'action_result') {
              setActiveActions(prev => {
                const newMap = new Map(prev);
                const existing = newMap.get(msg.goal_id);
                if (existing) {
                  newMap.set(msg.goal_id, {
                    ...existing,
                    status: 'completed',
                    result: msg.result
                  });
                }
                // Remove after 3 seconds
                setTimeout(() => {
                  setActiveActions(prev => {
                    const newMap = new Map(prev);
                    newMap.delete(msg.goal_id);
                    return newMap;
                  });
                }, 3000);
                return newMap;
              });
            } else if (msg.type === 'action_cancelled') {
              setActiveActions(prev => {
                const newMap = new Map(prev);
                const existing = newMap.get(msg.goal_id);
                if (existing) {
                  newMap.set(msg.goal_id, {
                    ...existing,
                    status: 'cancelled'
                  });
                }
                setTimeout(() => {
                  setActiveActions(prev => {
                    const newMap = new Map(prev);
                    newMap.delete(msg.goal_id);
                    return newMap;
                  });
                }, 2000);
                return newMap;
              });
            } else if (msg.type === 'action_error') {
              setActiveActions(prev => {
                const newMap = new Map(prev);
                const existing = newMap.get(msg.goal_id || 'unknown');
                if (existing) {
                  newMap.set(msg.goal_id || 'unknown', {
                    ...existing,
                    status: 'error'
                  });
                }
                return newMap;
              });
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

  const sendActionGoal = (action: string, goal: any) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      return;
    }

    wsRef.current.send(JSON.stringify({
      type: "send_action_goal",
      action: action,
      goal: goal
    }));
  };

  const cancelAction = (goalId: string) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      return;
    }

    wsRef.current.send(JSON.stringify({
      type: "cancel_action",
      goal_id: goalId
    }));
  };

  const handleNavigateToGoal = () => {
    sendActionGoal("navigate_to_goal", {
      x: parseFloat(navigateX),
      y: parseFloat(navigateY),
      theta: 0.0
    });
  };

  const handleFollowLine = () => {
    sendActionGoal("follow_line", {
      duration: parseFloat(followLineDuration)
    });
  };

  const handleObstacleAvoidance = () => {
    sendActionGoal("obstacle_avoidance", {
      direction: obstacleDirection,
      distance: 0.0  // 0 = infinite
    });
  };

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle>Autonomous Actions</CardTitle>
          <Badge variant={connected ? 'default' : 'secondary'}>
            {connected ? 'Connected' : 'Disconnected'}
          </Badge>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        {/* Navigate to Goal */}
        <div className="space-y-2">
          <Label>Navigate to Goal</Label>
          <div className="flex gap-2">
            <Input
              type="number"
              placeholder="X"
              value={navigateX}
              onChange={(e) => setNavigateX(e.target.value)}
              className="w-20"
            />
            <Input
              type="number"
              placeholder="Y"
              value={navigateY}
              onChange={(e) => setNavigateY(e.target.value)}
              className="w-20"
            />
            <Button
              onClick={handleNavigateToGoal}
              disabled={!connected}
              size="sm"
            >
              Go
            </Button>
          </div>
        </div>

        {/* Follow Line */}
        <div className="space-y-2">
          <Label>Follow Line</Label>
          <div className="flex gap-2">
            <Input
              type="number"
              placeholder="Duration (s)"
              value={followLineDuration}
              onChange={(e) => setFollowLineDuration(e.target.value)}
              className="w-32"
            />
            <Button
              onClick={handleFollowLine}
              disabled={!connected}
              size="sm"
            >
              Start
            </Button>
          </div>
        </div>

        {/* Obstacle Avoidance */}
        <div className="space-y-2">
          <Label>Obstacle Avoidance</Label>
          <div className="flex gap-2">
            <select
              value={obstacleDirection}
              onChange={(e) => setObstacleDirection(e.target.value)}
              className="px-3 py-2 border rounded"
            >
              <option value="forward">Forward</option>
              <option value="backward">Backward</option>
              <option value="left">Left</option>
              <option value="right">Right</option>
            </select>
            <Button
              onClick={handleObstacleAvoidance}
              disabled={!connected}
              size="sm"
            >
              Start
            </Button>
          </div>
        </div>

        {/* Active Actions */}
        {activeActions.size > 0 && (
          <div className="space-y-2 pt-4 border-t">
            <Label>Active Actions</Label>
            {Array.from(activeActions.values()).map((action) => (
              <div key={action.goal_id} className="flex items-center justify-between p-2 bg-muted rounded">
                <div>
                  <span className="font-medium">{action.action}</span>
                  <Badge variant="outline" className="ml-2">
                    {action.status}
                  </Badge>
                </div>
                {action.status === 'active' && (
                  <Button
                    onClick={() => cancelAction(action.goal_id)}
                    size="sm"
                    variant="destructive"
                  >
                    Cancel
                  </Button>
                )}
              </div>
            ))}
          </div>
        )}
      </CardContent>
    </Card>
  );
}

