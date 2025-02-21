import { Canvas } from "@react-three/fiber";
import { DirectionComponentProps } from "../../definitions/componentTypeDefinitions";
import CarModel from "./CarModel";
import { useEffect, useState } from "react";
import * as THREE from "three";
import WindParticles from "./WindParticles";
import { OrbitControls } from "@react-three/drei";
import { useSelector } from "react-redux";
import { RootState } from "../../store/mainStore";

const DirectionComponent = ({ initialAngle }: DirectionComponentProps) => {
  const [carModelSize, setCarModelSize] = useState<THREE.Vector3>(
    new THREE.Vector3()
  );
  const [carModelCenter, setCarModelCenter] = useState<THREE.Vector3>(
    new THREE.Vector3()
  );
  const [angle, setAngle] = useState(initialAngle);
  const [lightPosition, setLightPosition] = useState(
    new THREE.Vector3(5, 10, 5)
  );

  const movement = useSelector((state: RootState) => state.app.movementData);

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case "a":
          setAngle((prev) => prev + 10);
          break;
        case "d":
          setAngle((prev) => prev - 10);
          break;
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
        angle={angle}
      />
      {/* TODO: Stop animation*/}
      {!movement.angle && (
        <WindParticles
          carModelCenter={carModelCenter}
          carModelSize={carModelSize}
          numberOfParticles={6}
          angle={angle}
        />
      )}
      <OrbitControls />
    </Canvas>
  );
};

export default DirectionComponent;
