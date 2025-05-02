import { Canvas } from "@react-three/fiber";
import { DirectionComponentProps } from "../../definitions/componentTypeDefinitions";
import Scene from "./Scene";

const DirectionComponent = ({ initialAngle }: DirectionComponentProps) => {

  return (
    <Canvas style={{ height: "100%", width: "100%" }}>
      <Scene />
    </Canvas>
  );
};

export default DirectionComponent;
