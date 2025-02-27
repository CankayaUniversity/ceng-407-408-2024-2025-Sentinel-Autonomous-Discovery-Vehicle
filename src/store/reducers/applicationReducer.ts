import {
  ApplicationStateType,
  MovementDataType,
} from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";

const initialState: ApplicationStateType = {
  isAppBarOpen: false,
  movementData: {
    left_speed: 0,
    right_speed: 0,
    angle: null,
  },
  isDialogOpen: false,
  isCameraPlaying: false,
};

const applicationSlice = createSlice({
  name: "application",
  initialState,
  reducers: {
    setIsAppBarOpen(state, action: PayloadAction<boolean>) {
      state.isAppBarOpen = action.payload;
    },
    setMovementData(state, action: PayloadAction<MovementDataType>) {
      state.movementData = action.payload;
    },
    setIsDialogOpen: (state, action: PayloadAction<boolean>) => {
      return { ...state, isDialogOpen: action.payload };
    },
    setIsCameraPlaying: (state, action: PayloadAction<boolean>) => {
      return { ...state, isCameraPlaying: action.payload };
    },
  },
});

export const { setIsAppBarOpen, setMovementData, setIsDialogOpen, setIsCameraPlaying } = applicationSlice.actions;
export default applicationSlice.reducer;
