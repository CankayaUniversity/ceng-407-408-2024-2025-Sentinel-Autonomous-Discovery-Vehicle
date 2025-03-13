import { Box, CircularProgress } from "@mui/material";
import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { useRos } from "../../../utils/RosContext";

const TwoDimensionalMapComponent = () => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const [hasReceivedMap, setHasReceivedMap] = useState(false);
  const { ros } = useRos();

  useEffect(() => {
    const listener = new ROSLIB.Topic({
      ros: ros,
      name: "/map",
      messageType: "nav_msgs/msg/OccupancyGrid",
    });

    listener.subscribe((message: any) => {
      setHasReceivedMap(true);

      const canvas = canvasRef.current;
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      if (!ctx) return;

      const { width, height } = message.info;
      const data = message.data;

      if (canvas.width !== height || canvas.height !== width) {
        canvas.width = height;
        canvas.height = width;
      }

      const imageData = ctx.createImageData(height, width);

      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const index = y * width + x;

          // Apply 90-degree clockwise rotation
          const rotatedX = height - 1 - y;
          const rotatedY = x;
          const rotatedIndex = rotatedY * height + rotatedX;

          let value = data[index];
          let r = 93, g = 110, b = 108; // Blue color

          if (value === 100) {
            r = 0; g = 0; b = 0; // Occupied = Black
          } else if (value === 0) {
            r = 193; g = 193; b = 193; // Free space = Light gray
          }

          const pixelIndex = rotatedIndex * 4;
          imageData.data[pixelIndex] = r; // R
          imageData.data[pixelIndex + 1] = g; // G
          imageData.data[pixelIndex + 2] = b; // B
          imageData.data[pixelIndex + 3] = 255; // Alpha
        }
      }

      ctx.putImageData(imageData, 0, 0);
    });

    return () => listener.unsubscribe();
  }, [ros]);

  return (
    <Box sx={{ width: "100%", height: "100%", position: "relative" }}>
      {hasReceivedMap ? (
        <canvas
          ref={canvasRef}
          style={{
            width: "100%",
            height: "100%",
            transform: "rotate(0deg) scale(1, 1)",
          }}
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
    </Box>
  );
};

export default TwoDimensionalMapComponent;
