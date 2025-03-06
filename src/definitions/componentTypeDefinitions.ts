import React from "react";

export type ToggleButtonProps = {
  setOpen: React.Dispatch<React.SetStateAction<boolean>>;
  appBarStyles: any;
};

export interface DirectionComponentProps {
  initialAngle: number;
}
