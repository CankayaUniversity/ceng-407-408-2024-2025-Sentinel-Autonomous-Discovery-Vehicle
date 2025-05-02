// import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App.tsx'
import { Provider } from 'react-redux';
import { store } from './store/mainStore.ts';
import "./index.css";
import { RosProvider } from './utils/RosContext.tsx';

createRoot(document.getElementById('root')!).render(

  <Provider store={store}>
    <RosProvider>
      <App />
    </RosProvider>
  </Provider>
)
