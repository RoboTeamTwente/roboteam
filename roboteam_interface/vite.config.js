import { fileURLToPath, URL } from 'node:url'
import { defineConfig } from 'vite';
import vue from '@vitejs/plugin-vue';

export default defineConfig({
  clearScreen: false,
  server: {
    port: 8080,
    strictPort: true,
  },
  envPrefix: ['VITE_'],
  build: {
    target: ['es2021', 'chrome97', 'safari13'],
    minify: !process.env.DEBUG ? 'esbuild' : false,
    sourcemap: !!!process.env.DEBUG,
  },
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url))
    }
  },
  plugins: [
    vue()
  ],
});
