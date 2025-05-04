import { NotificationItem } from "./notificationTypeDefinitions";
import { StoredMapImage } from "./twoDimensionalMapTypeDefinitions";

export interface ApplicationStateType {
  isAppBarOpen: boolean;
  movementData: MovementDataType;
  isCameraDialogOpen: boolean;
  isMapDialogOpen: boolean;
  isCameraPlaying: boolean;
  isDetectFrameEnabled: boolean;
  generateReport: boolean;
  generatedMaps: StoredMapImage[];
  notifications: NotificationItem[];
  fetchObjectWithId: FetchObjectWithIdType;
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