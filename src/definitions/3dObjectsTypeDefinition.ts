import { Color, Group, Mesh, Vector3 } from "three";

export interface RobotModelProps {
  ref: React.RefObject<Group | null>;
  linearSpeed: React.RefObject<number>;
}

export interface RobotWheels {
  leftFront: Mesh | null;
  rightFront: Mesh | null;
  leftBack: Mesh | null;
  rightBack: Mesh | null;
}

export interface FollowCameraProps {
  target: React.RefObject<Group | null>;
}

export interface RoadProps {
  robotRef: React.RefObject<Group | null>;
  linearSpeed: React.RefObject<number>;
  pointDistance?: number;
  roadWidth?: number;
  maxPoints?: number;
  regressionWindow?: number;
  frontGap?: number;
}

export interface WindParticlesProps {
  robotRef: React.RefObject<Group | null>;
  linearSpeed: React.RefObject<number>;
  numberOfParticles: number;
}

export interface WindComponentProps {
  robotRef: React.RefObject<Group | null>;
  robotSize: React.RefObject<Vector3>;
  linearSpeed: React.RefObject<number>;
  height: number;
  position: number;
  lineWidth?: number;
  color?: string | Color | number;
  numberOfPoints?: number;
}
