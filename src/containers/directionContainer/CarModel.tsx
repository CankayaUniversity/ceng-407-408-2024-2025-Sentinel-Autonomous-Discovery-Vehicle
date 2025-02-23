import { useGLTF } from "@react-three/drei";
import { useFrame, useThree } from "@react-three/fiber";
import { useEffect, useRef } from "react";
import * as THREE from "three";
import { CarModelProps } from "../../definitions/componentTypeDefinitions";

const cameraPositionMultiplier: number = 1.2;

const CarModel: React.FC<CarModelProps> = ({
  angle,
  setCarModelCenter,
  setLightPosition,
  setCarModelSize,
}) => {
  const modelRef = useRef<THREE.Group | null>(null);
  const { scene } = useGLTF("/models/sentinel.glb");
  const { camera } = useThree();

  useEffect(() => {
    const box = new THREE.Box3().setFromObject(scene);
    const size = new THREE.Vector3();
    box.getSize(size);
    const center = new THREE.Vector3();
    box.getCenter(center);
    const position = camera.position.set(
      center.x + size.x * cameraPositionMultiplier,
      center.y + size.y * cameraPositionMultiplier * 0.65,
      center.z + size.z * cameraPositionMultiplier,
    );

    camera.lookAt(position);

    setLightPosition(position);
    setCarModelCenter(center);
    setCarModelSize(size);
  }, [scene, camera, setCarModelCenter, setCarModelSize, setLightPosition]);

  useFrame(() => {
    if (modelRef.current) {
      modelRef.current.rotation.y = THREE.MathUtils.degToRad(angle);
    }
  });

  return <primitive object={scene} ref={modelRef} />;
};

export default CarModel;
