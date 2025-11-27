import { NextApiRequest, NextApiResponse } from 'next';
import { buildGatewayUrl } from '@/config/gateway';

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'POST') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const gatewayUrl = buildGatewayUrl('/api/system/mode/set');
    
    const body = req.body;
    
    const response = await fetch(gatewayUrl, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(5000),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(errorData.detail || `System mode API responded with status: ${response.status}`);
    }

    const data = await response.json();
    res.status(200).json(data);
    
  } catch (error) {
    console.error('[system-mode-proxy] Error setting mode:', error);
    res.status(503).json({ 
      error: 'System mode API unavailable',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}

