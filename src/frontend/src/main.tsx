import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter, Routes, Route } from 'react-router-dom'
import App from './App'
import Table from './pages/Table'
import './index.css'

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
	<BrowserRouter>
	  <Routes>
		<Route path="/" element={<App />} />
		<Route path="table" element={<Table />} />
		
	  </Routes>
	</BrowserRouter>
  </React.StrictMode>
)
