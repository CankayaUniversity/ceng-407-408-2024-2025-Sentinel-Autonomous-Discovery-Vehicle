import {
  ApplicationStateType,
  MovementDataType,
} from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";

const initialState: ApplicationStateType = {
  isAppBarOpen: false,
  movementData: {
    linear: 0,
    angular: 0,
  },
  isCameraDialogOpen: false,
  isMapDialogOpen: false,
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
      state.movementData = {
        angular: action.payload.angular,
        linear: action.payload.linear,
      };
    },
    setIsCameraDialogOpen: (state, action: PayloadAction<boolean>) => {
      state.isCameraDialogOpen = action.payload;
    },
    setIsMapDialogOpen: (state, action: PayloadAction<boolean>) => {
      state.isMapDialogOpen = action.payload;
    },
    setIsCameraPlaying: (state, action: PayloadAction<boolean>) => {
      state.isCameraPlaying = action.payload;
    },
  },
});

export const {
  setIsAppBarOpen,
  setMovementData,
  setIsCameraDialogOpen,
  setIsMapDialogOpen,
  setIsCameraPlaying,
} = applicationSlice.actions;
export default applicationSlice.reducer;
