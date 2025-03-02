import { Box } from "@mui/material";
import React, { useEffect, useRef, useState } from "react";
import { Joystick } from "react-joystick-component";
import { IJoystickUpdateEvent } from "react-joystick-component/build/lib/Joystick";
import { RootState } from "../../store/mainStore";
import ROSLIB from "roslib";
import { useSelector, useDispatch } from "react-redux";
import { setMovementData } from "../../store/reducers/applicationReducer";

const JoystickControl: React.FC = () => {
  const joystickRef = useRef<Joystick>(null);

  const [coordinates, setCoordinates] = useState<any>({ x: 0, y: 0 });

  const [color, setColor] = useState<string>("#959595");

  const ros = useSelector((state: RootState) => state.app.ros);

  const dispatch = useDispatch();

  useEffect(() => {
    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/movement",
      messageType: "std_msgs/String",
    });

    listener.subscribe((message: any) => {
      const movementData = JSON.parse(message.data);
      dispatch(setMovementData(movementData));
      if (movementData.angle == null) {
        handleStop();
        return;
      }
      const rad = (movementData.angle * Math.PI) / 180;
      const x = Math.cos(rad);
      const y = Math.sin(rad);
      setCoordinates({ x, y });
    });

    return () => listener.unsubscribe();
  }, [ros, dispatch]);

  useEffect(() => {
    if (coordinates.x != 0 || coordinates.y != 0) {
      setColor("#1b4135");
    }
  }, [coordinates]);

  const handleMove = (stick: IJoystickUpdateEvent) => {
    setCoordinates({ x: stick.x, y: stick.y });
  };

  const handleStop = () => {
    setCoordinates({ x: 0, y: 0 });
    setColor("#959595");
  };

  return (
    <Box
      sx={{
        width: "100%",
        height: "100%",
        position: "relative",
        display: "flex",
        justifyContent: "center",
        alignItems: "center",
      }}
    >
      <Box
        sx={{
          position: "relative",
          width: "25rem",
          height: "20rem",
        }}
      >
        <img
          src="/joystick.png"
          style={{
            width: "100%",
            height: "100%",
            position: "absolute",
            top: "0",
            left: "0",
          }}
          alt="joystick"
        />

        <Box
          sx={{
            position: "absolute",
            top: "55.5%",
            left: "37.2%",
            transform: "translate(-56%, -37.2%)",
            border: "2px solid black",
            borderRadius: "50%",
          }}
        >
          <Joystick
            size={75}
            baseColor="#323232"
            stickColor={color}
            move={handleMove}
            stop={handleStop}
            ref={joystickRef}
            pos={coordinates}
          />
        </Box>

        <Box
          sx={{
            position: "absolute",
            top: "55.5%",
            right: "15.2%",
            transform: "translate(-56%, -37.2%)",
            border: "2px solid black",
            borderRadius: "50%",
          }}
        >
          <Joystick
            size={75}
            baseColor="#323232"
            stickColor="#959595"
            move={handleMove}
            stop={handleStop}
            disabled={true}
          />
        </Box>
      </Box>
    </Box>
  );
};

export default JoystickControl;
