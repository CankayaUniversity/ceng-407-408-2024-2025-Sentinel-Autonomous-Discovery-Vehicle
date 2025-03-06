import { createSlice } from "@reduxjs/toolkit";
import { ThemeType } from "../../definitions/themeDefinitions";

const initialState: ThemeType = {
  mode: "dark",
};

const themeSlice = createSlice({
  name: "theme",
  initialState,
  reducers: {
    toggleTheme: (state) => {
      state.mode = state.mode === "light" ? "dark" : "light";
    },
  },
});

export const { toggleTheme } = themeSlice.actions;
export default themeSlice.reducer;
