import { ThemeProvider, CssBaseline } from '@mui/material';
import { useSelector } from 'react-redux';
import type { AppState } from './store/mainStore';
import AppContainer from './containers/appContainer/AppContainer';

function App() {
  const theme = useSelector((state: AppState) => state.theme.theme);

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <AppContainer />
    </ThemeProvider>
  );
}

export default App;
