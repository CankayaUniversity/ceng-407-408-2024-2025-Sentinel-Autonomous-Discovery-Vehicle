import { StoredMapImage } from "./twoDimensionalMapTypeDefinitions";

export interface ApplicationStateType {
  isAppBarOpen: boolean;
  movementData: MovementDataType;
  isCameraDialogOpen: boolean;
  isMapDialogOpen: boolean;
  isCameraPlaying: boolean;
  generateReport: boolean;
  generatedMaps: StoredMapImage[];
}

export interface MovementDataType {
  linear: number;
  angular: number;
}
