import ROSLIB from "roslib";

export interface ApplicationStateType {
  isAppBarOpen: boolean;
  ros: ROSLIB.Ros;
  movementData: MovementDataType;
}

export interface MovementDataType {
  left_speed: number;
  right_speed: number;
  angle: number | null;
  old_angle: number | null;
}

