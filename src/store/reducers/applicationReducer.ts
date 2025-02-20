import ROSLIB from "roslib";
import { ApplicationStateType, MovementDataType } from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";

const initialState: ApplicationStateType = {
  isAppBarOpen: false,
  ros: new ROSLIB.Ros({ url: "ws://localhost:9090" }),
  movementData: {
    left_speed: 0,
    right_speed: 0,
    angle: null,
  }
};

const applicationSlice = createSlice({
  name: "application",
  initialState,
  reducers: {
    setIsAppBarOpen(state, action: PayloadAction<boolean>) {
      state.isAppBarOpen = action.payload;
    },
    setMovementData(state, action: PayloadAction<MovementDataType>) {
      console.info(action);

      state.movementData = action.payload;
    }
  },
});

export const { setIsAppBarOpen, setMovementData } = applicationSlice.actions;
export default applicationSlice.reducer;

