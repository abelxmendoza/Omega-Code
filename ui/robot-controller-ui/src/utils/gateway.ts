/**
 * Gateway URL Builder
 * Simple utility for building gateway URLs with environment variable support
 */

export function buildGatewayUrl(path: string): string {
  // For Vercel: use NEXT_PUBLIC_API_URL
  const base =
    process.env.NEXT_PUBLIC_API_URL ||
    "http://localhost:8080";

  // Ensure clean slashes
  return `${base.replace(/\/+$/, '')}/${path.replace(/^\/+/, '')}`;
}

