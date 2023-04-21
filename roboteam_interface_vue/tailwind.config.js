/** @type {import('tailwindcss').Config} */
module.exports = {
    content: [
        "./index.html",
        "./src/**/*.{vue,js,ts,jsx,tsx}",
    ],
    theme: {
        extend: {
            gridTemplateAreas: {
                'layout': [
                    'header header header header',
                    'nav    drag-x main   main',
                    'nav    drag-x drag-y drag-y',
                    'nav    drag-x footer footer',
                ],
            },
            gridTemplateColumns: {
                'layout': 'min(90vw, var(--left-bar-width)) 3px 1fr 1fr auto',
                'fluid-10': 'repeat(auto-fit, minmax(10rem, 1fr))',
                'fluid-12': 'repeat(auto-fit, minmax(12rem, 1fr))',
            },
            gridTemplateRows: {
                'layout': '3rem 1fr 3px min(90vh, var(--bottom-bar-height)) auto',
            },
        },
    },
    plugins: [
        require('@savvywombat/tailwindcss-grid-areas'),
        require("daisyui")
    ],
    daisyui: {
        themes: [
            {
                light: {
                    ...require("daisyui/src/colors/themes")["[data-theme=light]"],
                    '--padding-card': '1rem',
                    "primary": "#2563eb",
                    "secondary": "#6b7280",
                    "accent": "#5a1c74",
                    "neutral": "#3D4451",
                    "base-100": "#FFFFFF",
                    "info": "#37bdf8",
                    "success": "#22c55e",
                    "warning": "#facc17",
                    "error": "#dc2626",
                },
                dark: {
                    ...require("daisyui/src/colors/themes")["[data-theme=dark]"],
                    '--padding-card': '1rem',
                    "primary": "#2563eb",
                    "secondary": "#6b7280",
                    "accent": "#5a1c74",
                    "neutral": "#3D4451",
                    // "base-100": "#FFFFFF",
                    "info": "#37bdf8",
                    "success": "#22c55e",
                    "warning": "#facc17",
                    "error": "#dc2626",
                }
            }
        ]
    },
}

