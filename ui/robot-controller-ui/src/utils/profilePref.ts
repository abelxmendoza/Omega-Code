export type Profile = 'lan' | 'tailscale' | 'local';

const KEY = 'ui.activeProfile';

export function normalize(p?: string | null): Profile | null {
  const v = (p || '').toLowerCase();
  return v === 'lan' || v === 'tailscale' || v === 'local' ? (v as Profile) : null;
}

export function getStoredProfile(): Profile | null {
  if (typeof window === 'undefined') return null;
  return normalize(localStorage.getItem(KEY));
}

export function setStoredProfile(p: Profile) {
  if (typeof window !== 'undefined') localStorage.setItem(KEY, p);
}

/** Decide which profile to use at runtime: ?profile > localStorage > env */
export function resolveProfileFromEnv(envDefault: Profile): Profile {
  if (typeof window === 'undefined') return envDefault;
  const url = new URL(window.location.href);
  const qp = normalize(url.searchParams.get('profile'));
  if (qp) { setStoredProfile(qp); return qp; }
  return getStoredProfile() || envDefault;
}
