import {
  ApplicationStateType,
  MovementDataType,
  FetchObjectWithIdType,
} from "../../definitions/applicationTypeDefinitions";
import { createSlice, PayloadAction } from "@reduxjs/toolkit";
import { StoredMapImage } from "../../definitions/twoDimensionalMapTypeDefinitions";
import { NotificationItem } from "../../definitions/notificationTypeDefinitions";
import { objectData } from "../../definitions/reportGeneratorTypeDefinitions";
import { reportTemplateData } from "../../containers/reportGenerator/ReportTemplate";

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
  notifications: [],
  fetchObjectWithId: {
    id: "",
    fetchObject: false,
  },
  clickedNotificationObject: {
    id: "",
    url: "",
    class: "",
  },
  reportData: reportTemplateData,
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
    setGeneratedMapsToReport: (state, action: PayloadAction<StoredMapImage[]>) => {
      const updatedSections = state.reportData.content.imageSections.map(section => {
        if (section.title === 'Generated Maps') {
          return {
            ...section,
            images: action.payload
          };
        }
        return section;
      });

      state.reportData = {
        ...state.reportData,
        content: {
          ...state.reportData.content,
          imageSections: updatedSections
        }
      };
    },
    addGeneratedMapToReport: (state, action: PayloadAction<StoredMapImage>) => {
      try {
        const newMap = {
          topic: String(action.payload.topic || ""),
          palette: String(action.payload.palette || ""),
          dataUrl: String(action.payload.dataUrl || ""),
          timestamp: String(action.payload.timestamp || new Date().toISOString())
        };

        const updatedSections = state.reportData.content.imageSections.map(section => {
          if (section.title === 'Generated Maps') {
            const currentImages = [...(section.images as StoredMapImage[] || [])];
            return {
              ...section,
              images: [...currentImages, newMap]
            };
          }
          return section;
        });

        state.reportData = {
          ...state.reportData,
          content: {
            ...state.reportData.content,
            imageSections: updatedSections as any
          }
        };
      } catch (error) {
        console.error("Error in addGeneratedMapToReport reducer:", error);
      }
    },
    clearGeneratedMapsFromReport: (state) => {
      try {
        const updatedSections = state.reportData.content.imageSections.map(section => {
          if (section.title === 'Generated Maps') {
            return {
              ...section,
              images: [] as StoredMapImage[]
            };
          }
          return section;
        });

        state.reportData = {
          ...state.reportData,
          content: {
            ...state.reportData.content,
            imageSections: updatedSections as any
          }
        };
      } catch (error) {
        console.error("Error in clearGeneratedMapsFromReport reducer:", error);
      }
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
    setClickedNotificationObject: (state, action: PayloadAction<objectData>) => {
      state.clickedNotificationObject = action.payload;
    },
    resetClickedNotificationObject: (state) => {
      state.clickedNotificationObject = {
        id: "",
        url: "",
        class: ""
      };
    },
    setReportData: (state, action: PayloadAction<typeof reportTemplateData>) => {
      state.reportData = action.payload;
    },
    setReportObjectData: (state, action: PayloadAction<objectData[]>) => {
      const updatedSections = state.reportData.content.imageSections.map(section => {
        if (section.title === 'Summary of Detected Object Types') {
          return {
            ...section,
            images: action.payload
          };
        }
        return section;
      });

      state.reportData = {
        ...state.reportData,
        content: {
          ...state.reportData.content,
          imageSections: updatedSections
        }
      };
    },
    resetReportData: (state) => {
      state.reportData = JSON.parse(JSON.stringify(reportTemplateData));
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
  setIsDetectFrameEnabled,
  addNotification,
  removeNotification,
  clearNotifications,
  setFetchObjectWithId,
  resetFetchObjectFlag,
  setClickedNotificationObject,
  resetClickedNotificationObject,
  setGeneratedMapsToReport,
  addGeneratedMapToReport,
  clearGeneratedMapsFromReport,
  setReportData,
  setReportObjectData,
  resetReportData
} = applicationSlice.actions;
export default applicationSlice.reducer;