import { useCallback, useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import {
  DirectionComponentProps,
  WindParticle,
} from "../../definitions/componentTypeDefinitions";
import { RootState } from "../../store/mainStore";
import { useSelector } from "react-redux";

const cameraPositionMultiplier: number = 0.8; // Change this to adjust the camera position

const DirectionComponent = ({ initialAngle }: DirectionComponentProps) => {
  const canvasRef = useRef<HTMLDivElement | null>(null);
  const [angle, setAngle] = useState(initialAngle);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const [containerSize, setContainerSize] = useState({ width: 0, height: 0 });
  const angleRef = useRef(angle);
  const robotRef = useRef<THREE.Object3D | null>(null);
  const backgroundColor = useSelector((state: RootState) =>
    state.theme.theme.palette.mode === "dark"
      ? state.theme.theme.palette.background.default
      : state.theme.theme.palette.background.paper,
  );

  const modelCenterPositionRef = useRef<THREE.Vector3 | null>(null);
  const modelSizeRef = useRef<THREE.Vector3 | null>(null);
  const particleRef = useRef<WindParticle[]>([]);

  useEffect(() => {
    angleRef.current = angle;
  }, [angle]);

  useEffect(() => {
    if (canvasRef.current) {
      const resizeObserver = new ResizeObserver(() => {
        setContainerSize({
          width: canvasRef.current!.clientWidth - 0.5,
          height: canvasRef.current!.clientHeight,
        });
      });

      resizeObserver.observe(canvasRef.current);
      return () => resizeObserver.disconnect();
    }
  }, []);

  useEffect(() => {
    if (sceneRef.current) {
      sceneRef.current.background = new THREE.Color(backgroundColor);
    }
  }, [backgroundColor]);

  const createScene = useCallback(() => {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(backgroundColor);
    return scene;
  }, []);

  const createCamera = useCallback(() => {
    const camera = new THREE.PerspectiveCamera(
      75,
      containerSize.width / containerSize.height,
      0.1,
      1000,
    );
    return camera;
  }, [containerSize]);

  const createRenderer = useCallback(() => {
    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(containerSize.width, containerSize.height);
    renderer.setPixelRatio(window.devicePixelRatio);
    return renderer;
  }, [containerSize]);

  const loadModel = useCallback(async () => {
    const loader = new GLTFLoader();

    const gltf = await loader.loadAsync("/models/sentinel.glb");
    const model = gltf.scene;
    model.scale.set(1, 1, 1);
    robotRef.current = model;
    sceneRef.current!.add(model);

    const boundingBox = new THREE.Box3().setFromObject(model);

    const size = new THREE.Vector3();
    boundingBox.getSize(size);
    const center = new THREE.Vector3();
    boundingBox.getCenter(center);

    const offset = Math.max(size.x, size.y, size.z) * cameraPositionMultiplier;

    cameraRef!.current!.position.set(
      center.x + offset * 1,
      center.y + offset,
      center.z + offset * 1.3,
    );

    cameraRef.current!.lookAt(center);

    modelCenterPositionRef.current = center;
    modelSizeRef.current = size;
  }, []);

  const createParticle = (yPosition: number, position: number) => {
    const points = [
      new THREE.Vector3(
        modelCenterPositionRef.current!.x + modelSizeRef.current!.x * 0.5,
        modelCenterPositionRef.current!.y + yPosition / 2,
        modelCenterPositionRef.current!.z +
          modelSizeRef.current!.z * 0.25 * position,
      ),
      new THREE.Vector3(
        modelCenterPositionRef.current!.x + modelSizeRef.current!.x * 0.5,
        Math.min(
          modelCenterPositionRef.current!.y + yPosition / 2,
          modelCenterPositionRef.current!.y + modelSizeRef.current!.y / 2,
        ),
        modelCenterPositionRef.current!.z +
          modelSizeRef.current!.z * 0.5 * position,
      ),
      new THREE.Vector3(
        modelCenterPositionRef.current!.x,
        Math.min(
          modelCenterPositionRef.current!.y + yPosition / 2,
          modelCenterPositionRef.current!.y + modelSizeRef.current!.y / 2,
        ),
        modelCenterPositionRef.current!.z +
          modelSizeRef.current!.z * 0.5 * position,
      ),
      new THREE.Vector3(
        modelCenterPositionRef.current!.x - modelSizeRef.current!.x * 0.5,
        Math.min(
          modelCenterPositionRef.current!.y + yPosition / 3,
          modelCenterPositionRef.current!.y + modelSizeRef.current!.y / 2,
        ),
        modelCenterPositionRef.current!.z +
          modelSizeRef.current!.z * 0.5 * position,
      ),
    ];

    const curve = new THREE.CatmullRomCurve3(points);
    const tubeGeometry = new THREE.TubeGeometry(curve, 30, 0.001, 5, true);

    const material = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide,
    });

    const positions = new Float32Array(tubeGeometry.attributes.position.array);
    const growingGeometry = new THREE.BufferGeometry();
    growingGeometry.setAttribute(
      "position",
      new THREE.BufferAttribute(new Float32Array(positions.length), 3),
    );

    const windEffect = new THREE.Mesh(tubeGeometry, material);
    sceneRef.current!.add(windEffect);
    return {
      mesh: windEffect,
      progress: 0,
      speed: 0.005 + Math.random() * 0.01, // Random speed for each particle
      positions: [],
      originalPositions: positions,
      originalGeometry: tubeGeometry,
    };
  };

  useEffect(() => {
    const run = async () => {
      sceneRef.current = createScene();
      cameraRef.current = createCamera();
      rendererRef.current = createRenderer();

      if (canvasRef.current) {
        if (canvasRef.current.children.length > 0) {
          canvasRef.current.removeChild(canvasRef.current.children[0]);
        }
        canvasRef.current.appendChild(rendererRef.current.domElement);
      }

      await loadModel();

      for (let index = 0; index < 6; index++) {
        const yPosition = Math.random() / 10;

        particleRef.current.push(createParticle(yPosition, 1));
        particleRef.current.push(createParticle(yPosition, -1));
        particleRef.current.push(createParticle(yPosition * -1, 1));
        particleRef.current.push(createParticle(yPosition * -1, -1));
      }

      const light = new THREE.DirectionalLight(0xffffff, 1);
      light.position.set(5, 10, 5);
      sceneRef.current.add(light);
    };

    run();
  }, [createCamera, createRenderer, createScene, loadModel]);

  const updateParticle = (particle: WindParticle) => {
    const originalPositions = particle.originalPositions;
    const pointsToShow = Math.floor(
      originalPositions.length * particle.progress,
    );

    particle.positions = []; // Clear previous indices

    for (let i = 0; i < pointsToShow - 2; i++) {
      particle.positions.push(i, i + 1, i + 2);
    }

    particle.mesh.geometry.setIndex(particle.positions);
    particle.mesh.geometry.attributes.position.needsUpdate = true;

    particle.progress += particle.speed;
    if (particle.progress >= 1) {
      particle.progress = 0; // Reset to create continuous effect
    }
  };

  const clearParticle = (particle: WindParticle) => {
    particle.mesh.geometry.setIndex([]);
    particle.mesh.geometry.attributes.position.needsUpdate = true;
  };

  useEffect(() => {
    const animate = () => {
      requestAnimationFrame(animate);

      const scene = sceneRef.current;
      const camera = cameraRef.current;
      const renderer = rendererRef.current;

      if (scene && camera && renderer) {
        const rotation = THREE.MathUtils.degToRad(angleRef.current);

        if (robotRef.current) {
          robotRef.current.rotation.y = rotation;
        }

        particleRef.current.forEach((particle) => {
          particle.mesh.rotation.y = rotation;
          updateParticle(particle);
        });

        renderer.render(scene, camera);
      }
    };

    animate();
  }, []);

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      switch (event.key) {
        case "a":
          setAngle((prevAngle) => prevAngle + 10);
          break;
        case "d":
          setAngle((prevAngle) => prevAngle - 10);
          break;
      }
    };

    window.addEventListener("keydown", handleKeyDown);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
    };
  }, []);

  useEffect(() => {
    const renderer = rendererRef.current;
    const camera = cameraRef.current;

    if (renderer && camera && containerSize.width && containerSize.height) {
      renderer.setSize(containerSize.width, containerSize.height);

      camera.updateProjectionMatrix();
    }
  }, [containerSize]);

  return <div ref={canvasRef} style={{ width: "100%", height: "100%" }} />;
};

export default DirectionComponent;
