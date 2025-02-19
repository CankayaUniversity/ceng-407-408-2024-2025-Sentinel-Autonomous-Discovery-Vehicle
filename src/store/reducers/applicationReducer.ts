import ROSLIB from "roslib";
import { ApplicationStateType } from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";

const initialState: ApplicationStateType = {
  isAppBarOpen: false,
  ros: new ROSLIB.Ros({ url: "ws://localhost:9090" }),
};

const applicationSlice = createSlice({
  name: "application",
  initialState,
  reducers: {
    setIsAppBarOpen(state, action: PayloadAction<boolean>) {
      state.isAppBarOpen = action.payload;
    },
  },
});

export const { setIsAppBarOpen } = applicationSlice.actions;
export default applicationSlice.reducer;

