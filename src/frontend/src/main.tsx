// main.tsx or main.js (depending on your setup)
import React from 'react'
import ReactDOM from 'react-dom/client'
import { BrowserRouter, Routes, Route } from 'react-router-dom'
import App from './App'
<<<<<<< feat/modal
// import Table from './pages/Table'
=======
import Table from './pages/Table' // Adjust the path as needed
>>>>>>> feat/frontend
import './index.css'
import Teste from './pages/test'

ReactDOM.createRoot(document.getElementById('root') as HTMLElement).render(
  <React.StrictMode>
<<<<<<< feat/modal
	<BrowserRouter>
	  <Routes>
		<Route path="/" element={<App />} />
		{/* <Route path="table" element={<Table />} /> */}
		<Route path="teste" element={<Teste />} />
		
	  </Routes>
	</BrowserRouter>
=======
    <BrowserRouter>
      <Routes>
        <Route path="/control" element={<App />} />
        <Route path="/" element={<Table />} />
      </Routes>
    </BrowserRouter>
>>>>>>> feat/frontend
  </React.StrictMode>
)

