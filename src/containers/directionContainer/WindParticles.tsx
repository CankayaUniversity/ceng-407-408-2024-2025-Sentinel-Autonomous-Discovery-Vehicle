import * as THREE from "three";
import { useRef, useEffect, useMemo } from "react";
import { WindParticlesProps } from "../../definitions/3dObjectsTypeDefinition";
import WindModel from "../../components/windComponent/WindComponent";

const WindParticles: React.FC<WindParticlesProps> = ({
  robotRef,
  linearSpeed,
  numberOfParticles,
}) => {
  const robotSizeRef = useRef<THREE.Vector3>(new THREE.Vector3());
  useEffect(() => {
    if (!robotRef.current) return;
    const box = new THREE.Box3().setFromObject(robotRef.current);
    const size = new THREE.Vector3();
    box.getSize(size);
    robotSizeRef.current = size;
  }, [robotRef]);

  const windParticleState = useMemo(() => {
    const initialState = [];
    for (let index = 0; index < numberOfParticles; index++) {
      const height = Math.random() / 10;
      initialState[0 * numberOfParticles + index] = height;
      initialState[1 * numberOfParticles + index] = height;
      initialState[2 * numberOfParticles + index] = height;
      initialState[3 * numberOfParticles + index] = height;
    }
    return initialState;
  }, [numberOfParticles]);

  return (
    <>
      {Array.from({ length: numberOfParticles }).map((_, i) => (
        <>
          {/* 0, 4, 8, ... */}
          <WindModel
            key={0 * numberOfParticles + i}
            robotSize={robotSizeRef}
            robotRef={robotRef}
            linearSpeed={linearSpeed}
            position={1}
            height={windParticleState[0 * numberOfParticles + i]}
          />
          {/* 1, 5, 9, ... */}
          <WindModel
            key={1 * numberOfParticles + i}
            robotSize={robotSizeRef}
            robotRef={robotRef}
            linearSpeed={linearSpeed}
            position={1}
            height={windParticleState[1 * numberOfParticles + i] * -1}
          />
          {/* 2, 6, 10, ... */}
          <WindModel
            key={2 * numberOfParticles + i}
            robotSize={robotSizeRef}
            robotRef={robotRef}
            linearSpeed={linearSpeed}
            position={-1}
            height={windParticleState[2 * numberOfParticles + i]}
          />
          {/* 3, 7, 11, ... */}
          <WindModel
            key={3 * numberOfParticles + i}
            robotSize={robotSizeRef}
            robotRef={robotRef}
            linearSpeed={linearSpeed}
            position={-1}
            height={windParticleState[3 * numberOfParticles + i] * -1}
          />
        </>
      ))}
    </>
  );
};

export default WindParticles;
