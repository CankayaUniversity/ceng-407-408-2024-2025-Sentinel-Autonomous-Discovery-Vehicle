import { Mesh, TubeGeometry } from "three";

export type ToggleButtonProps = {
  setOpen: React.Dispatch<React.SetStateAction<boolean>>;
  appBarStyles: any;
};

export interface DirectionComponentProps {
  initialAngle: number;
}

export interface WindParticle {
  mesh: Mesh;
  progress: number;
  speed: number;
  positions: number[];
  originalPositions: Float32Array;
  originalGeometry: TubeGeometry;
}

