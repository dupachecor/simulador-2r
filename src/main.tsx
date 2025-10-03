import React from 'react'
import { createRoot } from 'react-dom/client'
import TwoRWebSimulator from './two-r/TwoRWebSimulator'

const root = createRoot(document.getElementById('root')!)
root.render(<React.StrictMode><TwoRWebSimulator /></React.StrictMode>)
