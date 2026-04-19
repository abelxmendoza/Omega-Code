import { NextApiRequest, NextApiResponse } from 'next';
import { buildGatewayUrl } from '@/config/gateway';

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const gatewayUrl = buildGatewayUrl('/api/system/mode/status');
    
    const response = await fetch(gatewayUrl, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
      signal: AbortSignal.timeout(2000),
    });

    if (!response.ok) {
      throw new Error(`System mode API responded with status: ${response.status}`);
    }

    const data = await response.json();
    res.status(200).json(data);
    
  } catch (error) {
    console.warn('[system-mode-proxy] Gateway unreachable:', error instanceof Error ? error.message : String(error));
    res.status(200).json({ mode: 'unknown', available: false });
  }
}

