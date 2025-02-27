import { Box, CircularProgress } from "@mui/material";
import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { useRos } from "../../../utils/RosContext";

const TwoDimensionalMapComponent = () => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);
  const [isDataStreaming, setIsDataStreaming] = useState<boolean>();

  const { ros } = useRos();

  useEffect(() => {

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/map",
      messageType: "nav_msgs/msg/OccupancyGrid",
    });

    listener.subscribe((message: any) => {
      setIsDataStreaming(true);
      const canvas = canvasRef.current;
      if (canvas && message) {
        const ctx = canvas.getContext("2d");
        if (ctx) {
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.fillText("Map Data Rendered Here", 10, 20);
        }
      }

      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }

      timeoutRef.current = setTimeout(() => {
        setIsDataStreaming(false);
      }, 2000);
    });

    return () => listener.unsubscribe();
  }, [ros]);

  return (
    <>
      {isDataStreaming ? (
        <canvas
          ref={canvasRef}
          style={{ border: "1px solid blue", width: "100%", height: "100%" }}
        />
      ) : (
        <Box
          sx={{
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
            height: "100%",
          }}
        >
          <CircularProgress />
        </Box>
      )}
    </>
  );
};

export default TwoDimensionalMapComponent;
