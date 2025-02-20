import ROSLIB from "roslib";

export interface ApplicationStateType {
  isAppBarOpen: boolean;
  ros: ROSLIB.Ros;
  movementData: MovementDataType
}


export interface MovementDataType {
  left_speed: number | null,
  right_speed: number | null,
  angle: number | null,
}