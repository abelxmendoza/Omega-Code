import { NextApiRequest, NextApiResponse } from 'next';
import { buildGatewayUrl } from '@/config/gateway';

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const gatewayUrl = buildGatewayUrl('/api/system/mode/list');
    
    const response = await fetch(gatewayUrl, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
      signal: AbortSignal.timeout(5000),
    });

    if (!response.ok) {
      throw new Error(`System mode API responded with status: ${response.status}`);
    }

    const data = await response.json();
    res.status(200).json(data);
    
  } catch (error) {
    console.error('[system-mode-proxy] Error fetching mode list:', error);
    res.status(503).json({ 
      error: 'System mode API unavailable',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}

