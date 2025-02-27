import { toggleTheme } from "../../../store/reducers/themeReducer";
import Button from "@mui/material/Button";
import DarkModeIcon from "@mui/icons-material/DarkMode";
import { RootState } from "../../../store/mainStore";
import LightModeIcon from "@mui/icons-material/LightMode";
import { useDispatch, useSelector } from "react-redux";

const DarkModeButton = () => {
  const dispatch = useDispatch();
  const themeMode = useSelector((state: RootState) => state.theme.mode);

  return (
    <Button
      sx={{ position: "absolute", top: "0.8rem", right: "0.8rem" }}
      variant="text"
      onClick={() => dispatch(toggleTheme())}
    >
      {themeMode === "dark" ? <LightModeIcon /> : <DarkModeIcon />}
    </Button>
  );
};

export default DarkModeButton;
