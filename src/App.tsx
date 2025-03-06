import { ThemeProvider, CssBaseline } from "@mui/material";
import type { RootState } from "./store/mainStore";
import AppContainer from "./containers/appContainer/AppContainer";
import { useSelector } from "react-redux";
import { lightTheme, darkTheme } from "./constants/themeConstants";

function App() {
  const themeMode = useSelector((state: RootState) => state.theme.mode);

  return (
    <ThemeProvider theme={themeMode === "dark" ? darkTheme : lightTheme}>
      <CssBaseline />
      <AppContainer />
    </ThemeProvider>
  );
}

export default App;
