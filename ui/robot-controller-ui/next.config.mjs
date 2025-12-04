/** @type {import('next').NextConfig} */
const nextConfig = {
    reactStrictMode: true,
    images: {
      remotePatterns: [
        {
          protocol: 'http',
          hostname: '**',
        },
      ],
    },
    logging: {
      fetches: {
        fullUrl: true,
      },
    },
    // Enhanced error handling and logging for Vercel builds
    eslint: {
      // Don't fail build on ESLint errors during Vercel builds
      // This allows the build to continue even if ESLint has issues
      ignoreDuringBuilds: false,
    },
    typescript: {
      // Don't fail build on TypeScript errors - let it show warnings instead
      // This helps identify issues without blocking deployment
      ignoreBuildErrors: false,
    },
    // Add verbose logging for build process
    onDemandEntries: {
      maxInactiveAge: 25 * 1000,
      pagesBufferLength: 2,
    },
    // Log build information
    webpack: (config, { buildId, dev, isServer, defaultLoaders, webpack }) => {
      // Add build information to help with debugging
      if (!dev) {
        console.log('[Vercel Build Debug] Build ID:', buildId);
        console.log('[Vercel Build Debug] Is Server:', isServer);
        console.log('[Vercel Build Debug] Node Environment:', process.env.NODE_ENV);
        console.log('[Vercel Build Debug] Vercel Environment:', process.env.VERCEL_ENV);
      }
      
      // Ignore Cypress config during webpack build
      config.resolve.alias = {
        ...config.resolve.alias,
        'cypress': false,
      };
      
      return config;
    },
    // Security headers - only in production
    async headers() {
      // Only add CSP headers in production
      // In development, Next.js handles this differently and CSP can cause issues
      if (process.env.NODE_ENV !== 'production') {
        return [];
      }
      
      return [
        {
          source: '/:path*',
          headers: [
            {
              key: 'Content-Security-Policy',
              value: [
                "default-src 'self'",
                "script-src 'self' 'unsafe-inline' 'unsafe-eval'", // Needed for Next.js
                "script-src-elem 'self' 'unsafe-inline'", // For inline script elements
                "style-src 'self' 'unsafe-inline'", // Needed for CSS-in-JS
                "font-src 'self' data: https://fonts.googleapis.com https://fonts.gstatic.com https://fonts-api.fontawesome.com",
                "img-src 'self' data: blob: http: https:",
                "connect-src 'self' http://localhost:* ws://localhost:* wss://localhost:* http://*:* ws://*:* wss://*:*",
                "frame-src 'self'",
                "object-src 'none'",
                "base-uri 'self'",
                "form-action 'self'",
                "frame-ancestors 'none'",
                "upgrade-insecure-requests",
              ].join('; '),
            },
          ],
        },
      ];
    },
  };
  
  export default nextConfig;