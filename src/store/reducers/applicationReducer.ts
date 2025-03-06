import {
  ApplicationStateType,
  MovementDataType,
} from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";

const roundSmallValues = (value: number, threshold = 1e-3) => {
  return Math.abs(value) < threshold ? 0 : value;
};

const initialState: ApplicationStateType = {
  isAppBarOpen: false,
  movementData: {
    left_speed: 0,
    right_speed: 0,
    angle: null,
    old_angle: null,
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
      const left_speed = roundSmallValues(action.payload.left_speed);
      const right_speed = roundSmallValues(action.payload.right_speed);
      const payloadAngle = action.payload.angle;
      const angle =
        left_speed == 0 && right_speed == 0
          ? null
          : payloadAngle
            ? (payloadAngle + 45) % 360
            : null;

      state.movementData = {
        old_angle: state.movementData.angle || state.movementData.old_angle,
        angle,
        left_speed,
        right_speed,
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

export const { setIsAppBarOpen, setMovementData, setIsCameraDialogOpen, setIsMapDialogOpen, setIsCameraPlaying } = applicationSlice.actions;
export default applicationSlice.reducer;
