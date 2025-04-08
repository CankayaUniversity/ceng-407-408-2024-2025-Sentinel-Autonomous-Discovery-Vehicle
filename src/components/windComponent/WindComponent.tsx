import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";
import React from "react";
import { WindComponentProps } from "../../definitions/3dObjectsTypeDefinition";

const WindModel: React.FC<WindComponentProps> = React.memo(
  ({
    robotRef,
    linearSpeed,
    robotSize,
    height,
    position,
    lineWidth = 1.5,
    numberOfPoints = 100,
    color = 0xa5d4ff,
  }) => {
    const randomNumber = (min: number, max: number) =>
      Math.floor(Math.random() * (max - min + 1) + min);

    const visiblePoints = useRef<number>(0);
    const speed = useRef<number>(randomNumber(1, 3));
    const lineRef = useRef<THREE.Line | null>(null);
    const stopPath = useRef<THREE.Vector3[] | null>(null);

    const zPosition = (offset: number) =>
      robotSize.current.z * offset * position - 0.02;

    const createPath = () => {
      const size = robotSize.current;
      const yPosition = size.y * 0.5 - size.y * height * 1.8 - 0.025;
      const offsets = [
        new THREE.Vector3(size.x * 0.5, yPosition, zPosition(0.2)),
        new THREE.Vector3(size.x * 0.45, yPosition, zPosition(0.4)),
        new THREE.Vector3(0, yPosition, zPosition(0.5)),
        new THREE.Vector3(-size.x * 0.5, yPosition, zPosition(0.5)),
      ];

      return offsets.map((offset) => robotRef.current!.localToWorld(offset));
    };

    const createGeometry = (path: THREE.Vector3[]) => {
      const points = new THREE.CatmullRomCurve3(path)
        .getPoints(numberOfPoints)
        .slice(0, visiblePoints.current);
      return new THREE.BufferGeometry().setFromPoints(points);
    };

    const updateVisiblePoints = (delta: number) => {
      if (visiblePoints.current >= numberOfPoints + 1) {
        visiblePoints.current = 0;
      }
      const newVisiblePoints =
        visiblePoints.current +
        speed.current * delta * 60 * linearSpeed.current * 2;
      visiblePoints.current = Math.max(0, newVisiblePoints);
      speed.current = randomNumber(1, 3);
    };

    useFrame((_, delta) => {
      if (!robotRef.current || !lineRef.current) return;
      if (delta > 0.1) return;
      if (linearSpeed.current == 0 && visiblePoints.current == 0) {
        stopPath.current = null;
        return;
      }

      let path: THREE.Vector3[];

      if (!stopPath.current) {
        path = createPath();
        if (linearSpeed.current === 0) {
          stopPath.current = path;
        } else {
          updateVisiblePoints(delta);
        }
      } else {
        const newVisiblePoints =
          visiblePoints.current - speed.current * delta * 120;
        visiblePoints.current = Math.max(0, newVisiblePoints);
        path = stopPath.current;
      }

      const geometry = createGeometry(path);
      if (lineRef.current.geometry) {
        lineRef.current.geometry.dispose();
      }
      lineRef.current.geometry = geometry;
    });

    return (
      <primitive object={new THREE.Line()} ref={lineRef}>
        <bufferGeometry />
        <lineBasicMaterial color={color} linewidth={lineWidth} />
      </primitive>
    );
  }
);

export default WindModel;
