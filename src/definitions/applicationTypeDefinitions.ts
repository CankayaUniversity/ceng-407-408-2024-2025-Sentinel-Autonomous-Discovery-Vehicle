
export interface ApplicationStateType {
  isAppBarOpen: boolean;
  movementData: MovementDataType;
  isCameraDialogOpen: boolean;
  isMapDialogOpen: boolean;
  isCameraPlaying: boolean;
}

export interface MovementDataType {
  left_speed: number;
  right_speed: number;
  angle: number | null;
  old_angle: number | null;
}

