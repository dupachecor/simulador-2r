import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  base: '/simulador-2r/', // cambia por el nombre de tu repo en GitHub
  plugins: [react()],
})
