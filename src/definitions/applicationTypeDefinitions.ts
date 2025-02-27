
export interface ApplicationStateType {
  isAppBarOpen: boolean;
  movementData: MovementDataType;
  isCameraDialogOpen: boolean;
  isMapDialogOpen: boolean;
  isCameraPlaying: boolean;
}


export interface MovementDataType {
  left_speed: number | null,
  right_speed: number | null,
  angle: number | null,
}