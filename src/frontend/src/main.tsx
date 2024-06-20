// main.tsx or main.js (depending on your setup)
import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter, Routes, Route } from 'react-router-dom'
import App from './App'
import Table from './pages/Table' // Adjust the path as needed
import './index.css'

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
    <BrowserRouter>
      <Routes>
        <Route path="/control/:id" element={<App />} />
        <Route path="/" element={<Table />} />
      </Routes>
    </BrowserRouter>
  </React.StrictMode>
)

