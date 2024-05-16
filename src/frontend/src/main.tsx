import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter, Routes, Route } from 'react-router-dom'
import App from './App'
import About from './pages/About'
import './index.css'

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
	<BrowserRouter>
	  <Routes>
		<Route path="/" element={<App />}>
		  <Route path="about" element={<About />} />
		</Route>
	  </Routes>
	</BrowserRouter>
  </React.StrictMode>
)
