import { ThemeProvider, CssBaseline } from "@mui/material";
import type { RootState } from "./store/mainStore";
import AppContainer from "./containers/appContainer/AppContainer";
import { useSelector } from "react-redux";

function App() {
  const theme = useSelector((state: RootState) => state.theme.theme);

  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <AppContainer />
    </ThemeProvider>
  );
}

export default App;
