import { createSlice } from "@reduxjs/toolkit";
import { darkTheme, lightTheme } from "../../constants/themeConstants";
import { ThemeType } from "../../definitions/themeDefinitions";

const initialState: ThemeType = {
  theme: darkTheme,
  mode: "dark",
};

const themeSlice = createSlice({
  name: "theme",
  initialState,
  reducers: {
    toggleTheme: (state) => {
      state.mode = state.mode === "light" ? "dark" : "light";
      (state.theme as any) = state.mode === "light" ? lightTheme : darkTheme;
    },
  },
});

export const { toggleTheme } = themeSlice.actions;
export default themeSlice.reducer;
