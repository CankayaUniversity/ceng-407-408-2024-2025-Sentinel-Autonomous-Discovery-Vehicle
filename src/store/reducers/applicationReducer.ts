import ROSLIB from "roslib";
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
  ros: new ROSLIB.Ros({ url: "ws://localhost:9090" }),
  movementData: {
    left_speed: 0,
    right_speed: 0,
    angle: null,
    old_angle: null,
  },
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
  },
});

export const { setIsAppBarOpen, setMovementData } = applicationSlice.actions;
export default applicationSlice.reducer;
