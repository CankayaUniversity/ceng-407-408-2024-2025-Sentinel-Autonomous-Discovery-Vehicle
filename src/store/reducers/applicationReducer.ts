import {
  ApplicationStateType,
  MovementDataType,
} from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";
import { StoredMapImage } from "../../definitions/twoDimensionalMapTypeDefinitions";

const initialState: ApplicationStateType = {
  isAppBarOpen: false,
  movementData: {
    linear: 0,
    angular: 0,
  },
  isCameraDialogOpen: false,
  isMapDialogOpen: false,
  isCameraPlaying: false,
  generateReport: false,
  generatedMaps: [],
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
    setGenerateReport: (state, action: PayloadAction<boolean>) => {
      state.generateReport = action.payload;
    },
    setGeneratedMaps: (state, action: PayloadAction<StoredMapImage[]>) => {
      state.generatedMaps = action.payload;
    },
    addGeneratedMap: (state, action: PayloadAction<StoredMapImage>) => {
      state.generatedMaps.push(action.payload);
    },
    clearGeneratedMaps: (state) => {
      state.generatedMaps = [];
    },
  },
});

export const {
  setIsAppBarOpen,
  setMovementData,
  setIsCameraDialogOpen,
  setIsMapDialogOpen,
  setIsCameraPlaying,
  setGenerateReport,
  setGeneratedMaps,
  addGeneratedMap,
  clearGeneratedMaps,
} = applicationSlice.actions;
export default applicationSlice.reducer;