import { useEffect, useRef } from "react";
import {
  RobotModelProps,
  RobotWheels,
} from "../../definitions/3dObjectsTypeDefinition";
import * as THREE from "three";
import { useGLTF } from "@react-three/drei";
import { useFrame } from "@react-three/fiber";
import { useSelector } from "react-redux";
import { RootState } from "../../store/mainStore";

const RobotModel: React.FC<RobotModelProps> = ({ ref, linearSpeed }) => {
  const { scene } = useGLTF("/models/sentinel.glb");
  const wheelsRef = useRef<RobotWheels>({
    leftFront: null,
    rightFront: null,
    leftBack: null,
    rightBack: null,
  });
  const position = useRef({ x: 0, y: 0, z: 0 });
  const yaw = useRef(0);
  const size = useRef<THREE.Vector3>(new THREE.Vector3());
  const wheelRadius = useRef(0.12);

  useEffect(() => {
    if (!ref.current) return;

    const box = new THREE.Box3().setFromObject(scene);
    const sizeModel = new THREE.Vector3();
    box.getSize(sizeModel);
    size.current = sizeModel;

    scene.children.forEach((child) => {
      if (child.name.includes("wheel1")) {
        wheelsRef.current.rightFront = child as THREE.Mesh;
      } else if (child.name.includes("wheel2")) {
        wheelsRef.current.leftFront = child as THREE.Mesh;
      } else if (child.name.includes("wheel3")) {
        wheelsRef.current.leftBack = child as THREE.Mesh;
      } else if (child.name.includes("wheel4")) {
        wheelsRef.current.rightBack = child as THREE.Mesh;
      }
    });
  }, [ref, scene]);

  const movement = useSelector((state: RootState) => state.app.movementData);

  useFrame((_, delta) => {
    if (!ref.current) return;
    if (!movement.angle) {
      linearSpeed.current = 0;
      return;
    }
    if (delta > 0.1) return;

    const { left_speed, right_speed } = movement;

    const linearSpeedValue = left_speed + right_speed;
    const angularSpeed = (right_speed - left_speed) / (size.current.x * 3);
    linearSpeed.current = linearSpeedValue;

    const deltaX = Math.cos(yaw.current) * linearSpeedValue * delta;
    const deltaZ = Math.sin(yaw.current) * linearSpeedValue * delta;
    const deltaYaw = angularSpeed * delta;

    position.current.x += deltaX;
    position.current.z -= deltaZ;
    yaw.current = (yaw.current + deltaYaw) % (2 * Math.PI);

    ref.current.position.x = position.current.x;
    ref.current.position.z = position.current.z;
    ref.current.rotation.y = yaw.current;

    const wheelAngularSpeedLeft = left_speed / wheelRadius.current;
    const wheelAngularSpeedRight = right_speed / wheelRadius.current;

    const { leftFront, rightFront, leftBack, rightBack } = wheelsRef.current;

    if (leftFront && rightFront && leftBack && rightBack) {
      leftFront.rotation.z += wheelAngularSpeedLeft * delta;
      leftBack.rotation.z += wheelAngularSpeedLeft * delta;
      rightFront.rotation.z += wheelAngularSpeedRight * delta;
      rightBack.rotation.z += wheelAngularSpeedRight * delta;
    }
  });

  return (
    <group ref={ref}>
      <primitive object={scene} />
    </group>
  );
};

export default RobotModel;
