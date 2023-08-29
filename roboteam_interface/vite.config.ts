import { fileURLToPath, URL } from 'node:url'
import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

export default ({ mode }) => {
  const isProduction = mode === 'production'

  return defineConfig({
    build: {
      target: ['es2021', 'chrome97', 'safari13'],
      minify: !isProduction ? 'esbuild' : false,
      sourcemap: true
    },
    clearScreen: false,
    plugins: [
      vue()
    ],
    resolve: {
      alias: {
        '@': fileURLToPath(new URL('./src', import.meta.url))
      }
    },
    server: {
      port: 8080,
      strictPort: true
    }
  })
};
