import React, { useRef } from "react";
import * as THREE from "three";
import { useFrame } from "@react-three/fiber";
import { RoadProps } from "../../definitions/3dObjectsTypeDefinition";
import { computePredictedPoint, createRoadGeometry } from "../../utils/road";

const Road: React.FC<RoadProps> = ({
  robotRef,
  linearSpeed,
  pointDistance = 0.01,
  roadWidth = 0.35,
  maxPoints = 50,
  regressionWindow = 2,
  frontGap = 0.6,
}) => {
  const pointsRef = useRef<THREE.Vector3[]>([]);
  const lastVelocityRef = useRef<THREE.Vector3>(new THREE.Vector3());
  const combinedMeshRef = useRef<THREE.Mesh>(null);
  const lastPositionRef = useRef<THREE.Vector3 | null>(null);

  useFrame((_, delta) => {
    if (!robotRef.current) return;
    if (delta > 0.1) return;
    if (linearSpeed.current == 0) return;

    const robot = robotRef.current;

    const robotPos = new THREE.Vector3(
      robot.position.x,
      robot.position.y - 0.026,
      robot.position.z,
    );

    if (lastPositionRef.current) {
      const instantVelocity = new THREE.Vector3().subVectors(
        robotPos,
        lastPositionRef.current,
      );
      lastVelocityRef.current.copy(instantVelocity);
    }

    lastPositionRef.current = robotPos.clone();

    const lastRecorded = pointsRef.current[pointsRef.current.length - 1];

    if (!lastRecorded || robotPos.distanceTo(lastRecorded) > pointDistance) {
      pointsRef.current.push(robotPos.clone());
      if (pointsRef.current.length > maxPoints) {
        pointsRef.current.shift();
      }
    }

    if (pointsRef.current.length < 2 || !combinedMeshRef.current) return;

    if (pointsRef.current.length >= regressionWindow) {
      computePredictedPoint(
        pointsRef.current,
        lastVelocityRef,
        regressionWindow,
      );
    }

    const allPoints = [...pointsRef.current];
    const lastPoint = pointsRef.current[pointsRef.current.length - 1];

    const normalizedVelocity = lastVelocityRef.current.clone().normalize();
    const endPoint = lastPoint
      .clone()
      .add(normalizedVelocity.multiplyScalar(frontGap));

    allPoints.push(endPoint);
    const combinedGeometry = createRoadGeometry(allPoints, roadWidth);
    combinedMeshRef.current.geometry.dispose();
    combinedMeshRef.current.geometry = combinedGeometry;
  });

  return (
    <group>
      <mesh ref={combinedMeshRef}>
        <meshStandardMaterial color={0x606060} side={THREE.DoubleSide} />
      </mesh>
    </group>
  );
};

export default Road;
