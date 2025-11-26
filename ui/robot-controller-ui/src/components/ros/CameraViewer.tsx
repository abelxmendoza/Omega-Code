/**
 * CameraViewer Component
 * 
 * Displays camera feed from ROS2 compressed image topic.
 * 
 * Usage:
 *   <CameraViewer topic="/camera/image_raw/compressed" />
 */

import React, { useEffect, useState, useRef } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Badge } from '@/components/ui/badge';
import { buildGatewayUrl } from '@/utils/gateway';

interface CameraViewerProps {
  topic?: string;
  className?: string;
  width?: number;
  height?: number;
}

export function CameraViewer({
  topic = '/camera/image_raw/compressed',
  className = '',
  width = 640,
  height = 480
}: CameraViewerProps) {
  const [connected, setConnected] = useState(false);
  const [fps, setFps] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const frameCountRef = useRef(0);
  const lastFpsTimeRef = useRef(Date.now());

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
          setError(null);

          // Subscribe to compressed image topic
          ws.send(JSON.stringify({
            type: 'subscribe',
            topic: topic,
            msg_type: 'CompressedImage'
          }));
        };

        ws.onmessage = (event) => {
          if (!mounted) return;
          try {
            const msg = JSON.parse(event.data);

            if (msg.type === 'ros2_message' && msg.topic === topic) {
              // Decode base64 image
              if (msg.data && msg.data.data) {
                const imageData = msg.data.data;
                const canvas = canvasRef.current;
                
                if (canvas) {
                  // Create image from base64
                  if (!imgRef.current) {
                    imgRef.current = new Image();
                    imgRef.current.onload = () => {
                      const ctx = canvas.getContext('2d');
                      if (ctx) {
                        canvas.width = imgRef.current!.width;
                        canvas.height = imgRef.current!.height;
                        ctx.drawImage(imgRef.current!, 0, 0);
                      }
                    };
                  }
                  
                  imgRef.current.src = `data:image/jpeg;base64,${imageData}`;
                  
                  // Update FPS
                  frameCountRef.current++;
                  const now = Date.now();
                  if (now - lastFpsTimeRef.current >= 1000) {
                    setFps(frameCountRef.current);
                    frameCountRef.current = 0;
                    lastFpsTimeRef.current = now;
                  }
                }
              }
            } else if (msg.type === 'subscription_confirmed') {
              console.log(`Subscribed to ${topic}`);
            } else if (msg.type === 'error') {
              setError(msg.data?.error || 'Unknown error');
            }
          } catch (e) {
            console.error('Error parsing message:', e);
          }
        };

        ws.onerror = (error) => {
          if (!mounted) return;
          console.error('WebSocket error:', error);
          setError('Connection error');
          setConnected(false);
        };

        ws.onclose = () => {
          if (!mounted) return;
          setConnected(false);
          wsRef.current = null;
        };
      } catch (err) {
        if (!mounted) return;
        setError(err instanceof Error ? err.message : 'Connection failed');
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
  }, [topic]);

  return (
    <Card className={className}>
      <CardHeader className="pb-2">
        <div className="flex items-center justify-between">
          <CardTitle className="text-sm font-medium">Camera Feed</CardTitle>
          <div className="flex gap-2">
            <Badge variant={connected ? 'default' : 'secondary'}>
              {connected ? 'Connected' : 'Disconnected'}
            </Badge>
            {fps > 0 && (
              <Badge variant="outline">{fps} FPS</Badge>
            )}
          </div>
        </div>
      </CardHeader>
      <CardContent>
        {error ? (
          (() => {
            console.error('CameraViewer Error:', error);
            return null;
          })()
        ) : (
          <div className="relative bg-black rounded-lg overflow-hidden" style={{ width, height }}>
            <canvas
              ref={canvasRef}
              className="w-full h-full object-contain"
              style={{ maxWidth: '100%', maxHeight: '100%' }}
            />
            {!connected && (
              <div className="absolute inset-0 flex items-center justify-center text-white">
                Connecting...
              </div>
            )}
          </div>
        )}
        <div className="text-xs text-muted-foreground mt-2">
          Topic: {topic}
        </div>
      </CardContent>
    </Card>
  );
}

