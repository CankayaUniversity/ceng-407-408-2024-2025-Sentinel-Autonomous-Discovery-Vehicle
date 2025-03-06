import { useFrame, useThree } from "@react-three/fiber";
import { FollowCameraProps } from "../../definitions/3dObjectsTypeDefinition";
import * as THREE from "three";
import { useRef } from "react";
import { OrbitControls } from "@react-three/drei";

const FollowCamera: React.FC<FollowCameraProps> = ({ target }) => {
  const { camera } = useThree();
  const offset = useRef<THREE.Vector3>(new THREE.Vector3(-0.5, 0.35, 0));

  useFrame(() => {
    if (!target.current) return;

    // const targetPosition = target.current.position;
    // const rotatedOffset = offset.current
    //   .clone()
    //   .applyQuaternion(target.current.quaternion);
    // const newPosition = new THREE.Vector3()
    //   .copy(targetPosition)
    //   .add(rotatedOffset);
    // camera.position.set(newPosition.x, newPosition.y, newPosition.z);
    // camera.lookAt(targetPosition);

    const targetPosition = target.current.position;
    const newPosition = new THREE.Vector3(
      targetPosition.x + offset.current.x,
      targetPosition.y + offset.current.y,
      targetPosition.z + offset.current.z,
    );
    camera.position.set(newPosition.x, newPosition.y, newPosition.z);
    camera.lookAt(targetPosition);
  });

  return <OrbitControls />;
};

export default FollowCamera;
