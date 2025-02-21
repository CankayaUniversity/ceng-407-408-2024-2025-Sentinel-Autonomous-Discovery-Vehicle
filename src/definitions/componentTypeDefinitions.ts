import { Vector3, Color } from "three";

export type ToggleButtonProps = {
  setOpen: React.Dispatch<React.SetStateAction<boolean>>;
  appBarStyles: any;
};

export interface DirectionComponentProps {
  initialAngle: number;
}

export interface CarModelProps {
  setCarModelCenter: React.Dispatch<React.SetStateAction<Vector3>>;
  setCarModelSize: React.Dispatch<React.SetStateAction<Vector3>>;
  setLightPosition: React.Dispatch<React.SetStateAction<Vector3>>;
  angle: number;
}

export interface WindComponentProps {
  carModelSize: Vector3;
  carModelCenter: Vector3;
  height: number;
  position: number;
  visiblePoint: number;
  angle: number;
  lineWidth?: number;
  color?: string | Color | number;
  numberOfPoints?: number;
}

export interface WindParticlesProps {
  carModelSize: Vector3;
  carModelCenter: Vector3;
  numberOfParticles: number;
  angle: number;
}

export interface WindParticleState {
  visiblePoints: number;
  speed: number;
  height: number;
}
