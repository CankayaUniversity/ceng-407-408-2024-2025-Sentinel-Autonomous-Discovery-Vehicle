import { Line } from "@react-three/drei";
import { useMemo } from "react";
import * as THREE from "three";
import React from "react";
import { WindComponentProps } from "../../definitions/componentTypeDefinitions";

const WindModel: React.FC<WindComponentProps> = React.memo(
  ({
    carModelCenter,
    carModelSize,
    height,
    position,
    visiblePoint,
    angle,
    lineWidth = 1,
    numberOfPoints = 100,
    color = 0xa5d4ff,
  }) => {
    const path = useMemo(() => {
      return [
        new THREE.Vector3(
          carModelCenter.x + carModelSize.x * 0.5,
          carModelCenter.y + height * 0.41205,
          carModelCenter.z + carModelSize.z * 0.25 * position
        ),
        new THREE.Vector3(
          carModelCenter.x + carModelSize.x * 0.5,
          carModelCenter.y + height * 0.45,
          carModelCenter.z + carModelSize.z * 0.5 * position
        ),
        new THREE.Vector3(
          carModelCenter.x,
          carModelCenter.y + height * 0.4,
          carModelCenter.z + carModelSize.z * 0.5 * position
        ),
        new THREE.Vector3(
          carModelCenter.x - carModelSize.x * 0.5,
          carModelCenter.y + height * 0.33,
          carModelCenter.z + carModelSize.z * 0.5 * position
        ),
      ];
    }, [carModelCenter, carModelSize, height, position]);

    const fullPath = useMemo(() => {
      return new THREE.CatmullRomCurve3(path).getPoints(numberOfPoints);
    }, [path, numberOfPoints]);

    return (
      <Line
        points={fullPath.slice(0, visiblePoint)}
        color={color}
        lineWidth={lineWidth}
        dashed={false}
        rotation={[0, THREE.MathUtils.degToRad(angle), 0]}
      />
    );
  }
);

export default WindModel;
