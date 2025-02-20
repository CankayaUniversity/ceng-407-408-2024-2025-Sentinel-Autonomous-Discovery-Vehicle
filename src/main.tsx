import { createRoot } from 'react-dom/client'
import App from './App.tsx'
import { Provider } from 'react-redux';
import { store } from './store/mainStore.ts';
import "./index.css";

createRoot(document.getElementById('root')!).render(

  <Provider store={store}>
    <App />
  </Provider>
)
