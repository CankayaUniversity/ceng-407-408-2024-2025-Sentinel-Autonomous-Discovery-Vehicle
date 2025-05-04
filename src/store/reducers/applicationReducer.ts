import {
  ApplicationStateType,
  MovementDataType,
  FetchObjectWithIdType
} from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";
import { StoredMapImage } from "../../definitions/twoDimensionalMapTypeDefinitions";
import { NotificationItem } from "../../definitions/notificationTypeDefinitions";

const initialState: ApplicationStateType = {
  isAppBarOpen: false,
  movementData: {
    linear: 0,
    angular: 0,
  },
  isCameraDialogOpen: false,
  isMapDialogOpen: false,
  isCameraPlaying: false,
  isDetectFrameEnabled: false,
  generateReport: false,
  generatedMaps: [],
  notifications: [],
  fetchObjectWithId: {
    id: "",
    fetchObject: false,
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
    setIsDetectFrameEnabled: (state, action: PayloadAction<boolean>) => {
      state.isDetectFrameEnabled = action.payload;
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
    addNotification: (state, action: PayloadAction<NotificationItem>) => {
      state.notifications.unshift(action.payload);
    },
    removeNotification: (state, action: PayloadAction<string>) => {
      state.notifications = state.notifications.filter(
        notification => notification.id !== action.payload
      );
    },
    clearNotifications: (state) => {
      state.notifications = [];
    },
    setFetchObjectWithId: (state, action: PayloadAction<FetchObjectWithIdType>) => {
      state.fetchObjectWithId = action.payload;
    },
    resetFetchObjectFlag: (state) => {
      state.fetchObjectWithId = {
        ...state.fetchObjectWithId,
        fetchObject: false
      };
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
  setIsDetectFrameEnabled,
  clearGeneratedMaps,
  addNotification,
  removeNotification,
  clearNotifications,
  setFetchObjectWithId,
  resetFetchObjectFlag,
} = applicationSlice.actions;
export default applicationSlice.reducer;