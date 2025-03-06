import { Box, CircularProgress } from '@mui/material';
import { useEffect, useRef, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from '../../utils/RosContext';

const LidarComponent = () => {
    const canvasRef = useRef<HTMLCanvasElement | null>(null);
    const timeoutRef = useRef<NodeJS.Timeout | null>(null);
    const [isDataStreaming, setIsDataStreaming] = useState<boolean>();

    const { ros } = useRos();

    useEffect(() => {
        const listener = new ROSLIB.Topic({
            ros: ros,
            name: '/scan',
            messageType: 'sensor_msgs/LaserScan',
        });

        // Handle incoming messages
        listener.subscribe((message: any) => {
            setIsDataStreaming(true);
            const canvas = canvasRef.current;
            if (canvas && message.ranges) {
                const ctx = canvas.getContext('2d');
                if (ctx) {
                    ctx.clearRect(0, 0, canvas.width, canvas.height);

                    const width = canvas.width;
                    const height = canvas.height;
                    const centerX = width / 2;
                    const centerY = height / 2;
                    const scale = 100;

                    // Draw each LiDAR point
                    ctx.fillStyle = 'red';
                    message.ranges.forEach((range: number, index: number) => {
                        if (range > 0 && range < message.range_max) {
                            // Convert polar coordinates to Cartesian
                            const angle = message.angle_min + index * message.angle_increment;
                            const x = centerX + range * scale * Math.cos(angle);
                            const y = centerY + range * scale * Math.sin(angle);

                            // Draw a small circle for each LiDAR point
                            ctx.beginPath();
                            ctx.arc(x, y, 2, 0, 2 * Math.PI);
                            ctx.fill();
                        }
                    });
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
        <> {
            isDataStreaming ? (
                <canvas
                    ref={canvasRef}
                    style={{ width: "100%", height: "100%" }}
                    width="600"
                    height="600"
                />
            ) : <Box
                sx={{
                    display: "flex",
                    justifyContent: "center",
                    alignItems: "center",
                    height: "100%",
                }}>
                <CircularProgress />
            </Box>}
        </>
    );
};

export default LidarComponent;
