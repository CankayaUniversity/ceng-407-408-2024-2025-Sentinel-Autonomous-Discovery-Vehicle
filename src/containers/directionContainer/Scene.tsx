import { useRef } from "react";
import * as THREE from "three";
import RobotModel from "./RobotModel";
import FollowCamera from "./FollowCamera";
import Road from "./Road";
import WindParticles from "./WindParticles";

const Scene = () => {
  const robotModel = useRef<THREE.Group | null>(null);
  const linearSpeed = useRef<number>(0);

  return (
    <>
      <ambientLight intensity={0.5} />
      <directionalLight position={[10, 10, 10]} />
      <RobotModel ref={robotModel} linearSpeed={linearSpeed} />
      <Road robotRef={robotModel} linearSpeed={linearSpeed} />
      <FollowCamera target={robotModel} />
      <WindParticles
        robotRef={robotModel}
        numberOfParticles={6}
        linearSpeed={linearSpeed}
      />
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.028, 0]}>
        <planeGeometry args={[200, 200, 200, 200]} />
        <meshStandardMaterial wireframe={true} color={"gray"} />
      </mesh>
    </>
  );
};

// <gridHelper args={[200, 200]} color={"gray"} />;
export default Scene;
