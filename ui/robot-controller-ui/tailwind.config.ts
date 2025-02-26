// File: tailwind.config.ts
// Summary:
// Configures TailwindCSS for the project, specifying content sources, 
// extending the theme with custom gradients, and preparing for future plugin integrations.

import type { Config } from "tailwindcss";

const config: Config = {
  content: [
    "./src/pages/**/*.{js,ts,jsx,tsx,mdx}", // Pages directory
    "./src/components/**/*.{js,ts,jsx,tsx,mdx}", // Components directory
    "./src/app/**/*.{js,ts,jsx,tsx,mdx}", // App directory for Next.js 13+
  ],
  theme: {
    extend: {
      backgroundImage: {
        "gradient-radial": "radial-gradient(var(--tw-gradient-stops))", // Radial gradients
        "gradient-conic":
          "conic-gradient(from 180deg at 50% 50%, var(--tw-gradient-stops))", // Conic gradients
      },
      colors: {
        neonBlue: "#00f", // Example custom color
        lightGray: "#f8f9fa", // Light gray background
      },
    },
  },
  plugins: [],
};

export default config;
