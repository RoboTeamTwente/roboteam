import { fileURLToPath, URL } from 'node:url'
import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

export default ({ mode }) => {
  const isProduction = mode === 'production'

  return defineConfig({
    clearScreen: false,
    server: {
      port: 8080,
      strictPort: true
    },
    build: {
      target: ['es2021', 'chrome97', 'safari13'],
      minify: !isProduction ? 'esbuild' : false,
      sourcemap: true
    },
    resolve: {
      alias: {
        '@': fileURLToPath(new URL('./src', import.meta.url))
      }
    },
    plugins: [
      vue(),
    ]
  })
};
