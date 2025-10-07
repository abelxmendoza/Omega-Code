import { NextApiRequest, NextApiResponse } from 'next';
import { getActiveProfile } from '@/utils/resolveWsUrl';

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const profile = getActiveProfile();
    
    // Build the gateway URL based on the active profile
    let gatewayUrl: string;
    switch (profile) {
      case 'tailscale':
        gatewayUrl = `http://${process.env.NEXT_PUBLIC_ROBOT_HOST_TAILSCALE}:7070/api/performance/metrics`;
        break;
      case 'lan':
        gatewayUrl = `http://${process.env.NEXT_PUBLIC_ROBOT_HOST_LAN}:7070/api/performance/metrics`;
        break;
      case 'local':
        gatewayUrl = `http://${process.env.NEXT_PUBLIC_ROBOT_HOST_LOCAL}:7070/api/performance/metrics`;
        break;
      default:
        gatewayUrl = `http://omega1.local:7070/api/performance/metrics`;
    }

    console.log(`[performance-proxy] Fetching metrics from: ${gatewayUrl}`);
    
    const response = await fetch(gatewayUrl, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
      // Add timeout to prevent hanging
      signal: AbortSignal.timeout(5000),
    });

    if (!response.ok) {
      throw new Error(`Performance API responded with status: ${response.status}`);
    }

    const data = await response.json();
    res.status(200).json(data);
    
  } catch (error) {
    console.error('[performance-proxy] Error fetching metrics:', error);
    res.status(503).json({ 
      error: 'Performance API unavailable',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}
