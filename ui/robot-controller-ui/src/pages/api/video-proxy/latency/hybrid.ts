import { NextApiRequest, NextApiResponse } from 'next';

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Get video stream URL from query params or env
    const profile = (req.query.profile as string) || 'local';
    const videoStreamUrl = process.env[`NEXT_PUBLIC_VIDEO_STREAM_URL_${profile.toUpperCase()}`] 
      || process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL
      || 'http://localhost:5000';
    
    // Extract base URL (remove /video_feed if present)
    const baseUrl = videoStreamUrl.replace(/\/video_feed.*$/, '');
    const latencyUrl = `${baseUrl}/latency/hybrid`;
    
    const response = await fetch(latencyUrl, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
      signal: AbortSignal.timeout(5000),
    });

    if (!response.ok) {
      // 503 is expected when hybrid system is not available
      if (response.status === 503) {
        const data = await response.json().catch(() => ({}));
        return res.status(503).json(data);
      }
      throw new Error(`Hybrid latency API responded with status: ${response.status}`);
    }

    const data = await response.json();
    res.status(200).json(data);
    
  } catch (error) {
    console.error('[latency-proxy] Error fetching hybrid latency:', error);
    res.status(503).json({ 
      error: 'Hybrid latency API unavailable',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}

