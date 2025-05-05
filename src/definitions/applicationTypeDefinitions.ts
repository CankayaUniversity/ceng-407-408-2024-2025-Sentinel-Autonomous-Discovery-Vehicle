import { reportTemplateData } from "../containers/reportGenerator/ReportTemplate";
import { NotificationItem } from "./notificationTypeDefinitions";
import { objectData } from "./reportGeneratorTypeDefinitions";

export interface ApplicationStateType {
  isAppBarOpen: boolean;
  movementData: MovementDataType;
  isCameraDialogOpen: boolean;
  isMapDialogOpen: boolean;
  isCameraPlaying: boolean;
  isDetectFrameEnabled: boolean;
  generateReport: boolean;
  notifications: NotificationItem[];
  fetchObjectWithId: FetchObjectWithIdType;
  clickedNotificationObject: objectData;
  reportData: typeof reportTemplateData;
  isGeneratingMaps: boolean;
  isFetchingObjects: boolean;
}

export interface FetchObjectWithIdType {
  id: string;
  fetchObject: boolean;
}

export interface MovementDataType {
  linear: number;
  angular: number;
}

export interface NotificationListProps {
  notifications: NotificationItem[];
}