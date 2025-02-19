import { useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader.js";
import { DirectionComponentProps } from "../../definitions/componentTypeDefinitions";
import { RootState } from "../../store/mainStore";
import { useSelector } from "react-redux";

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
      : state.theme.theme.palette.background.paper
  );

  const cameraPositionMultiplier: number = 0.8; // Change this to adjust the camera position

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

  useEffect(() => {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(backgroundColor);
    sceneRef.current = scene;

    const camera = new THREE.PerspectiveCamera(
      75,
      containerSize.width / containerSize.height,
      0.1,
      1000
    );
    cameraRef.current = camera;

    const renderer = new THREE.WebGLRenderer();
    renderer.setSize(containerSize.width, containerSize.height);
    renderer.setPixelRatio(window.devicePixelRatio);
    rendererRef.current = renderer;

    if (canvasRef.current) {
      if (canvasRef.current.children.length > 0) {
        canvasRef.current.removeChild(canvasRef.current.children[0]);
      }
      canvasRef.current.appendChild(renderer.domElement);
    }

    const loader = new GLTFLoader();
    loader.load(
      "/models/sentinel.glb",
      (gltf) => {
        const model = gltf.scene;
        model.scale.set(1, 1, 1);
        robotRef.current = model;
        scene.add(model);

        const boundingBox = new THREE.Box3().setFromObject(model);

        const size = new THREE.Vector3();
        boundingBox.getSize(size);
        const center = new THREE.Vector3();
        boundingBox.getCenter(center);

        // Adjust the camera position to be at the top-right corner
        const offset =
          Math.max(size.x, size.y, size.z) * cameraPositionMultiplier;

        camera.position.set(
          center.x + offset * 1,
          center.y + offset,
          center.z + offset * 1.3
        );

        // Make sure the camera is looking at the model center
        camera.lookAt(center);
      },
      undefined,
      (error) => {
        console.error("An error occurred while loading the model:", error);
      }
    );

    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 5);
    scene.add(light);

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
  }, [containerSize]);

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

        renderer.render(scene, camera);
      }
    };

    animate();
  }, []);

  useEffect(() => {
    const renderer = rendererRef.current;
    const camera = cameraRef.current;

    if (renderer && camera && containerSize.width && containerSize.height) {
      renderer.setSize(containerSize.width, containerSize.height);

      camera.updateProjectionMatrix();
    }
  }, [containerSize]);

  return (
    <>
      <div ref={canvasRef} style={{ width: "100%", height: "100%" }} />
    </>
  );
};

export default DirectionComponent;
