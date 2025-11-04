// utils/rosApi.ts
/**
 * ROS 2 API utilities for interacting with ROS Docker management endpoints
 */

import { buildGatewayUrl } from '@/config/gateway';

export interface ContainerStatus {
  Name: string;
  State: string;
  Status: string;
}

export interface ROSStatus {
  containers: ContainerStatus[];
  topics: string[];
  compose_path?: string;
}

export interface ContainerAction {
  action: 'start' | 'stop' | 'restart';
  service?: string;
}

/**
 * Fetch ROS 2 container status
 */
export async function getROSStatus(): Promise<ROSStatus> {
  const url = await buildGatewayUrl('/api/ros/status');
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to fetch ROS status: ${response.statusText}`);
  }
  return response.json();
}

/**
 * Control ROS 2 containers (start/stop/restart)
 */
export async function controlROSContainer(action: ContainerAction): Promise<{ success: boolean; message?: string; error?: string }> {
  const url = await buildGatewayUrl('/api/ros/control');
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(action),
  });
  
  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || error.error || `Failed to ${action.action} container`);
  }
  
  return response.json();
}

/**
 * Fetch logs from a ROS 2 container
 */
export async function getROSLogs(service: string, tail: number = 50): Promise<string[]> {
  const url = await buildGatewayUrl(`/api/ros/logs/${service}?tail=${tail}`);
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to fetch logs: ${response.statusText}`);
  }
  const data = await response.json();
  return data.logs || [];
}

/**
 * List ROS 2 topics
 */
export async function listROSTopics(): Promise<string[]> {
  const url = await buildGatewayUrl('/api/ros/topics');
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Failed to fetch topics: ${response.statusText}`);
  }
  const data = await response.json();
  return data.topics || [];
}

/**
 * Create WebSocket connection for ROS telemetry
 */
export function createROSTelemetryWebSocket(
  onMessage: (data: any) => void,
  onError?: (error: Event) => void,
  onClose?: () => void
): Promise<WebSocket> {
  return new Promise(async (resolve, reject) => {
    try {
      const wsUrl = await buildGatewayUrl('/ws/ros/telemetry');
      const protocol = wsUrl.startsWith('https') ? 'wss' : 'ws';
      const url = wsUrl.replace(/^https?/, protocol);
      
      const ws = new WebSocket(url);
      
      ws.onopen = () => resolve(ws);
      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          onMessage(data);
        } catch {
          onMessage({ topic: '/omega/telemetry', data: event.data });
        }
      };
      ws.onerror = onError || (() => {});
      ws.onclose = onClose || (() => {});
    } catch (error) {
      reject(error);
    }
  });
}

