export interface ApplicationStateType {
  isAppBarOpen: boolean;
  movementData: MovementDataType;
  isCameraDialogOpen: boolean;
  isMapDialogOpen: boolean;
  isCameraPlaying: boolean;
}

export interface MovementDataType {
  linear: number;
  angular: number;
}
