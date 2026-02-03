/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,ts,jsx,tsx}'],
  theme: {
    extend: {
      animation: {
        'pulse-glow': 'pulse-glow 2s ease-in-out infinite',
      },
      keyframes: {
        'pulse-glow': {
          '0%, 100%': { boxShadow: '0 0 8px rgba(251, 191, 36, 0.4)' },
          '50%': { boxShadow: '0 0 20px rgba(251, 191, 36, 0.7)' },
        },
      },
    },
  },
  plugins: [],
};
