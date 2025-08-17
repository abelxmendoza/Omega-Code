// File: /Omega-Code/ui/src/utils/netProfile.ts
// Summary:
// Tiny resolver for picking the right NEXT_PUBLIC_* URL by the active profile.
// Use in components to avoid hardcoding specific IPs.

type Profile = 'lan' | 'tailscale' | 'local';
const profile = (process.env.NEXT_PUBLIC_NETWORK_PROFILE as Profile) || 'local';

function pick(base: string) {
  const key = `${base}_${profile.toUpperCase()}`.replace(/-/g, '_');
  // @ts-ignore
  const val = process.env[key] as string | undefined;
  if (!val) console.warn(`[netProfile] Missing env for ${key}`);
  return val || '';
}

export const urls = {
  profile: () => profile,
  video: () => pick('NEXT_PUBLIC_VIDEO_STREAM_URL'),
  ws: {
    movement: () => pick('NEXT_PUBLIC_BACKEND_WS_URL_MOVEMENT'),
    ultrasonic: () => pick('NEXT_PUBLIC_BACKEND_WS_URL_ULTRASONIC'),
    line: () => pick('NEXT_PUBLIC_BACKEND_WS_URL_LINE_TRACKER'),
    lighting: () => pick('NEXT_PUBLIC_BACKEND_WS_URL_LIGHTING'),
    location: () => pick('NEXT_PUBLIC_BACKEND_WS_URL_LOCATION'),
  },
};

