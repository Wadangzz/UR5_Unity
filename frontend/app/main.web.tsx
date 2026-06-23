import React from 'react';
import ReactDOM from 'react-dom/client';
import { BrowserRouter, Route, Routes } from 'react-router-dom';
import './app.css';
import 'katex/dist/katex.min.css';
import App from '@/root';
import Slides from '@/components/Slides';
import RequireSim from '@/components/RequireSim';

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <BrowserRouter>
      <Routes>
        <Route
          path='/'
          element={
            <RequireSim>
              <App />
            </RequireSim>
          }
        />
        <Route path='/slides' element={<Slides />} />
      </Routes>
    </BrowserRouter>
  </React.StrictMode>,
);
