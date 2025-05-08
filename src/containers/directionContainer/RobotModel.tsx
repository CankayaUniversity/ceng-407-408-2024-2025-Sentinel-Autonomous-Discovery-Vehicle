import { useEffect, useRef } from "react";
import {
  RobotModelProps,
  RobotWheels,
} from "../../definitions/3dObjectsTypeDefinition";
import * as THREE from "three";
import { useGLTF } from "@react-three/drei";
import { useRos } from "../../utils/RosContext";
import ROSLIB from "roslib";
import { useDispatch, useSelector } from "react-redux";
import { RootState } from "../../store/mainStore";
import { setObjectIdOdom } from "../../store/reducers/applicationReducer";

const quaternionToYaw = (q: {
  x: number;
  y: number;
  z: number;
  w: number;
}): number => {
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return Math.atan2(siny_cosp, cosy_cosp);
};

const RobotModel: React.FC<RobotModelProps> = ({ ref, linearSpeed }) => {
  const { scene } = useGLTF("/models/sentinel.glb");
  const wheelsRef = useRef<RobotWheels>({
    leftFront: null,
    rightFront: null,
    leftBack: null,
    rightBack: null,
  });
  const size = useRef<THREE.Vector3>(new THREE.Vector3());
  const wheelRadius = useRef(0.035);
  const lastTimestampRef = useRef<number | null>(null);
  const dispatch = useDispatch();
  const notifications = useSelector((state: RootState) => state.app.notifications);
  const latestOdomRef = useRef<any>(null);

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

  const { ros } = useRos();

  useEffect(() => {
    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/odom",
      messageType: "nav_msgs/msg/Odometry",
    });

    listener.subscribe((message: any) => {
      if (!ref.current) return;

      const currentTime =
        message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;

      latestOdomRef.current = message;

      let deltaTime = 0;
      if (lastTimestampRef.current !== null) {
        deltaTime = currentTime - lastTimestampRef.current;
      }
      lastTimestampRef.current = currentTime;

      if (deltaTime === 0) return;

      const { position: pos, orientation: quat } = message.pose.pose;
      const { linear, angular } = message.twist.twist;

      ref.current.position.x = pos.x;
      ref.current.position.z = -pos.y;
      ref.current.rotation.y = quaternionToYaw(quat);
      linearSpeed.current = linear.x;

      const leftSpeed = linear.x - (angular.z * 0.128) / 2;
      const rightSpeed = linear.x + (angular.z * 0.128) / 2;

      const wheelAngularSpeedLeft = leftSpeed / wheelRadius.current;
      const wheelAngularSpeedRight = rightSpeed / wheelRadius.current;

      const wheelAngularDeltaLeft = wheelAngularSpeedLeft * deltaTime;
      const wheelAngularDeltaRight = wheelAngularSpeedRight * deltaTime;

      const { leftFront, rightFront, leftBack, rightBack } = wheelsRef.current;
      if (leftFront && rightFront && leftBack && rightBack) {
        leftFront.rotation.z += wheelAngularDeltaLeft;
        leftBack.rotation.z += wheelAngularDeltaLeft;
        rightFront.rotation.z += wheelAngularDeltaRight;
        rightBack.rotation.z += wheelAngularDeltaRight;
      }
    });

    return () => listener.unsubscribe();
  }, [ros, ref, linearSpeed]);

  useEffect(() => {
    if (notifications.length > 0) {
      const latest = notifications[0];
      if (latest.data.includes("New object detected") && latestOdomRef.current) {
        let idOdomMap = {
          id: latest.id,
          odom: latestOdomRef.current,
        }
        dispatch(setObjectIdOdom(idOdomMap));
      }
    }
  }, [notifications]);

  return (
    <group ref={ref}>
      <primitive object={scene} />
    </group>
  );
};

export default RobotModel;
