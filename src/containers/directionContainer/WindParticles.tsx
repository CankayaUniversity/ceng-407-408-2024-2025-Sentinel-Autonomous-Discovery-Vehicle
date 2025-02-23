import { useMemo, useRef, useState } from "react";
import { useFrame } from "@react-three/fiber";
import {
  WindParticlesProps,
  WindParticleState,
} from "../../definitions/componentTypeDefinitions";
import WindModel from "../../components/windComponent/WindComponent";

const randomNumber = (min: number, max: number) =>
  Math.floor(Math.random() * (max - min + 1) + min);

const FPS: number = 60;

const WindParticles: React.FC<WindParticlesProps> = ({
  numberOfParticles,
  carModelCenter,
  carModelSize,
  angle,
}) => {
  const numberOfPoints = useRef(100);

  const initialState = useMemo(() => {
    const initialState = [];
    for (let index = 0; index < numberOfParticles; index++) {
      const height = Math.random() / 10;

      initialState[0 * numberOfParticles + index] = {
        height,
        speed: randomNumber(1, 3),
        visiblePoints: 2,
      };

      initialState[1 * numberOfParticles + index] = {
        height,
        speed: randomNumber(1, 3),
        visiblePoints: 2,
      };
      initialState[2 * numberOfParticles + index] = {
        height,
        speed: randomNumber(1, 3),
        visiblePoints: 2,
      };
      initialState[3 * numberOfParticles + index] = {
        height,
        speed: randomNumber(1, 3),
        visiblePoints: 2,
      };
    }
    return initialState;
  }, [numberOfParticles]);

  const [windParticleState, setWindParticleState] =
    useState<WindParticleState[]>(initialState);

  useFrame((_, delta) => {
    setWindParticleState((prevArr) =>
      prevArr.map((prev) =>
        prev.visiblePoints < numberOfPoints.current + 1
          ? {
              ...prev,
              speed: randomNumber(1, 3),
              visiblePoints: prev.visiblePoints + prev.speed * delta * FPS,
            }
          : { ...prev, visiblePoints: 2 },
      ),
    );
  });

  return (
    <>
      {Array.from({ length: numberOfParticles }).map((_, i) => (
        <>
          {/* 0, 4, 8, ... */}
          <WindModel
            key={0 * numberOfParticles + i}
            carModelCenter={carModelCenter}
            carModelSize={carModelSize}
            angle={angle}
            position={1}
            numberOfPoints={numberOfPoints.current}
            height={windParticleState[0 * numberOfParticles + i].height}
            visiblePoint={
              windParticleState[0 * numberOfParticles + i].visiblePoints
            }
          />
          {/* 1, 5, 9, ... */}
          <WindModel
            key={1 * numberOfParticles + i}
            carModelCenter={carModelCenter}
            carModelSize={carModelSize}
            angle={angle}
            position={1}
            numberOfPoints={numberOfPoints.current}
            height={windParticleState[1 * numberOfParticles + i].height * -1}
            visiblePoint={
              windParticleState[1 * numberOfParticles + i].visiblePoints
            }
          />
          {/* 2, 6, 10, ... */}
          <WindModel
            key={2 * numberOfParticles + i}
            carModelCenter={carModelCenter}
            carModelSize={carModelSize}
            angle={angle}
            position={-1}
            numberOfPoints={numberOfPoints.current}
            height={windParticleState[2 * numberOfParticles + i].height}
            visiblePoint={
              windParticleState[2 * numberOfParticles + i].visiblePoints
            }
          />
          {/* 3, 7, 11, ... */}
          <WindModel
            key={3 * numberOfParticles + i}
            carModelCenter={carModelCenter}
            carModelSize={carModelSize}
            angle={angle}
            position={-1}
            numberOfPoints={numberOfPoints.current}
            height={windParticleState[3 * numberOfParticles + i].height * -1}
            visiblePoint={
              windParticleState[3 * numberOfParticles + i].visiblePoints
            }
          />
        </>
      ))}
    </>
  );
};

export default WindParticles;
