import { NextApiRequest, NextApiResponse } from 'next';
import { buildGatewayUrl } from '@/config/gateway';

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    const gatewayUrl = buildGatewayUrl('/api/performance/cache');

    console.log(`[performance-proxy] Fetching cache stats from: ${gatewayUrl}`);
    
    const response = await fetch(gatewayUrl, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
      // Add timeout to prevent hanging
      signal: AbortSignal.timeout(5000),
    });

    if (!response.ok) {
      throw new Error(`Cache API responded with status: ${response.status}`);
    }

    const data = await response.json();
    res.status(200).json(data);
    
  } catch (error) {
    console.error('[performance-proxy] Error fetching cache stats:', error);
    res.status(503).json({ 
      error: 'Cache API unavailable',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}
