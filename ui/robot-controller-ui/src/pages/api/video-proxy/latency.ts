import { NextApiRequest, NextApiResponse } from 'next';

export default async function handler(req: NextApiRequest, res: NextApiResponse) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  try {
    // Get video base URL from env or fallback to video stream URL
    const profile = (req.query.profile as string) || 'local';
    const videoBaseUrl = process.env.NEXT_PUBLIC_VIDEO_BASE_URL 
      || process.env[`NEXT_PUBLIC_VIDEO_STREAM_URL_${profile.toUpperCase()}`]?.replace(/\/video_feed.*$/, '')
      || process.env.NEXT_PUBLIC_VIDEO_STREAM_URL_LOCAL?.replace(/\/video_feed.*$/, '')
      || 'http://localhost:5000';
    
    // Extract base URL (remove /video_feed if present)
    const baseUrl = videoBaseUrl.replace(/\/video_feed.*$/, '');
    const latencyUrl = `${baseUrl}/latency`;
    
    const response = await fetch(latencyUrl, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
      signal: AbortSignal.timeout(5000),
    });

    if (!response.ok) {
      throw new Error(`Latency API responded with status: ${response.status}`);
    }

    const data = await response.json();
    res.status(200).json(data);
    
  } catch (error) {
    console.error('[latency-proxy] Error fetching latency:', error);
    res.status(503).json({ 
      error: 'Latency API unavailable',
      message: error instanceof Error ? error.message : 'Unknown error'
    });
  }
}

