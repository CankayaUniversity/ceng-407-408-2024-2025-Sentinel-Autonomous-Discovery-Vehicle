import Box from "@mui/material/Box";
import DarkModeButton from "../../components/buttons/darkModeButton/DarkModeButton";
import Grid from "@mui/material/Grid2";
import AppBarContainer from "../appBarContainer/AppBarContainer";
import {
  closeAppBarStyles,
  dataGridStyles,
} from "../../constants/styleConstants";
import { RootState } from "../../store/mainStore";
import HamburgerMenuButton from "../../components/buttons/hamburgerMenuButton/HamburgerMenuButton";
import { useEffect, useState } from "react";
import CameraContainer from "../cameraContainer/CameraContainer";
import MapContainer from "../mapContainer/MapContainer";
import {
  darkThemeBorderColor,
  lightThemeBorderColor,
} from "../../constants/themeConstants";
import DirectionContainer from "../directionContainer/DirectionContainer";
import { useSelector } from "react-redux";
import MovementContainer from "../movementContainer/MovementContainer";
import CameraFullScreenButton from "../cameraContainer/CameraFullScreenButton";

const AppContainer = () => {
  const isAppbarOpen = useSelector(
    (state: RootState) => state.app.isAppBarOpen
  );
  const paletteMode = useSelector((state: RootState) => state.theme.mode);
  const [appBarGridSize, setAppBarGridSize] = useState<number>();
  const [dataGridSize, setDataGridSize] = useState<number>();
  const [borderColor, setBorderColor] = useState<string>();

  useEffect(() => {
    if (isAppbarOpen) {
      setAppBarGridSize(2);
      setDataGridSize(10);
    } else {
      setAppBarGridSize(0);
      setDataGridSize(12);
    }
  }, [isAppbarOpen]);

  useEffect(() => {
    if (paletteMode === "dark") {
      setBorderColor(darkThemeBorderColor);
    } else {
      setBorderColor(lightThemeBorderColor);
    }
  }, [paletteMode]);

  return (
    <Box>
      <DarkModeButton />
      <HamburgerMenuButton appBarStyles={closeAppBarStyles} />
      <Grid container>
        <Grid size={appBarGridSize}>
          <AppBarContainer />
        </Grid>
        <Grid size={dataGridSize}>
          <Grid
            container
            spacing={2}
            sx={{ padding: isAppbarOpen ? "5rem 3rem 0 0" : "5rem" }}
          >
            <Grid size={{ xs: 12, lg: 6 }}>
              <Box
                sx={{ border: `1px solid ${borderColor}`, ...dataGridStyles }}
              >
                <div className="container" style={{ position: "relative" }}>
                  <CameraFullScreenButton />
                  <CameraContainer />
                </div>
              </Box>
            </Grid>
            <Grid size={{ xs: 12, lg: 6 }}>
              <Box
                sx={{ border: `1px solid ${borderColor}`, ...dataGridStyles }}
              >
                <MapContainer />
              </Box>
            </Grid>
            <Grid size={{ xs: 12, lg: 6 }}>
              <Box
                sx={{ border: `1px solid ${borderColor}`, ...dataGridStyles }}
              >
                <MovementContainer />
              </Box>
            </Grid>
            <Grid size={{ xs: 12, lg: 6 }}>
              <Box
                sx={{ border: `1px solid ${borderColor}`, ...dataGridStyles }}
              >
                <DirectionContainer initialAngle={0} />
              </Box>
            </Grid>
          </Grid>
        </Grid>
      </Grid>
    </Box>
  );
};

export default AppContainer;
