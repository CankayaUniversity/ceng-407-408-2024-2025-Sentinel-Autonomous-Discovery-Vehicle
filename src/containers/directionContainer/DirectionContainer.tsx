import { Canvas } from "@react-three/fiber";
import { DirectionComponentProps } from "../../definitions/componentTypeDefinitions";
import CarModel from "./CarModel";
import { useEffect, useState } from "react";
import * as THREE from "three";
import WindParticles from "./WindParticles";
import { OrbitControls } from "@react-three/drei";
import { useDispatch, useSelector } from "react-redux";
import { RootState } from "../../store/mainStore";
import { setMovementData } from "../../store/reducers/applicationReducer";

const DirectionComponent = ({ initialAngle }: DirectionComponentProps) => {
  const [carModelSize, setCarModelSize] = useState<THREE.Vector3>(
    new THREE.Vector3()
  );
  const [carModelCenter, setCarModelCenter] = useState<THREE.Vector3>(
    new THREE.Vector3()
  );

  const [lightPosition, setLightPosition] = useState(
    new THREE.Vector3(5, 10, 5)
  );

  const movement = useSelector((state: RootState) => state.app.movementData);
  const dispatch = useDispatch();

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case "a":
          dispatch(
            setMovementData({
              left_speed: 1,
              right_speed: 1,
              angle: movement.angle ? movement.angle + 10 : initialAngle + 10,
            } as any)
          );
          break;
        case "d":
          dispatch(
            setMovementData({
              left_speed: 1,
              right_speed: 1,
              angle: movement.angle ? movement.angle - 10 : initialAngle - 10,
            } as any)
          );
          break;
        default:
          dispatch(
            setMovementData({
              left_speed: 0,
              right_speed: 0,
              angle: null,
            } as any)
          );
      }
    };

    window.addEventListener("keydown", handleKeyDown);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, []);

  return (
    <Canvas style={{ height: "100%", width: "100%" }}>
      <directionalLight position={lightPosition} color={0xffffff} />
      <CarModel
        setCarModelCenter={setCarModelCenter}
        setCarModelSize={setCarModelSize}
        setLightPosition={setLightPosition}
        angle={movement.angle || movement.old_angle || initialAngle}
      />
      {movement.angle && (
        <WindParticles
          carModelCenter={carModelCenter}
          carModelSize={carModelSize}
          numberOfParticles={12}
          angle={movement.angle}
        />
      )}
      <OrbitControls />
    </Canvas>
  );
};

export default DirectionComponent;
