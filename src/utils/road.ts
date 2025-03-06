import * as THREE from "three";

const computePredictedPoint = (
  points: THREE.Vector3[],
  lastVelocityRef: React.RefObject<THREE.Vector3>,
  regressionWindow: number,
): THREE.Vector3 | null => {
  const n = Math.min(regressionWindow, points.length);
  if (n < 2) return null;

  const startIndex = points.length - n;
  const sumDelta = new THREE.Vector3();
  for (let i = startIndex + 1; i < points.length; i++) {
    const delta = new THREE.Vector3().subVectors(points[i], points[i - 1]);
    sumDelta.add(delta);
  }
  const averageDelta = sumDelta.divideScalar(n - 1);
  lastVelocityRef.current.copy(averageDelta);

  return points[points.length - 1].clone().add(averageDelta);
};

const createRoadGeometry = (points: THREE.Vector3[], roadWidth: number) => {
  if (points.length < 2) return new THREE.BufferGeometry();

  const vertices: number[] = [];
  const indices: number[] = [];

  for (let i = 0; i < points.length; i++) {
    const directionVector = new THREE.Vector3();
    if (i === 0) {
      directionVector.subVectors(points[1], points[0]).normalize();
    } else if (i === points.length - 1) {
      directionVector.subVectors(points[i], points[i - 1]).normalize();
    } else {
      directionVector.subVectors(points[i + 1], points[i - 1]).normalize();
    }

    const sideVector = new THREE.Vector3()
      .crossVectors(directionVector, new THREE.Vector3(0, 1, 0))
      .normalize();

    const leftPoint = points[i]
      .clone()
      .addScaledVector(sideVector, roadWidth / 2);
    const rightPoint = points[i]
      .clone()
      .addScaledVector(sideVector, -roadWidth / 2);

    vertices.push(leftPoint.x, leftPoint.y, leftPoint.z);
    vertices.push(rightPoint.x, rightPoint.y, rightPoint.z);
  }

  for (let i = 0; i < points.length - 1; i++) {
    const indexLeft1 = i * 2;
    const indexRight1 = i * 2 + 1;
    const indexLeft2 = (i + 1) * 2;
    const indexRight2 = (i + 1) * 2 + 1;
    indices.push(indexLeft1, indexRight1, indexLeft2);
    indices.push(indexRight1, indexRight2, indexLeft2);
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(vertices, 3),
  );
  geometry.setIndex(indices);
  geometry.computeVertexNormals();
  return geometry;
};

export { computePredictedPoint, createRoadGeometry };
