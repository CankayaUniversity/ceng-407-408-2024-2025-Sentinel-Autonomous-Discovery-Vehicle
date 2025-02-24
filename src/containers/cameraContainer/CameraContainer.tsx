import React, { useEffect, useState, useRef } from "react";
import ROSLIB from "roslib";
import CircularProgress from "@mui/material/CircularProgress";
import { Box, IconButton } from "@mui/material";
import PlayCircleIcon from "@mui/icons-material/PlayCircle";
import PauseCircleIcon from "@mui/icons-material/PauseCircle";
import "./CameraContainer.css";
import { RootState } from "../../store/mainStore";
import { useSelector } from "react-redux";
import { dataGridStyles } from "../../constants/styleConstants";
import CameraFullScreenButton from "./CameraFullScreenButton";

const CameraContainer: React.FC<{ isFullscreen?: boolean }> = ({ isFullscreen = false }) => {
    const [imageData, setImageData] = useState<string | null>(null);
    const [isPlaying, setIsPlaying] = useState<boolean>(false);
    const [isHovered, setIsHovered] = useState<boolean>(false);
    const timeoutRef = useRef<NodeJS.Timeout | null>(null);
    const ros = useSelector((state: RootState) => state.app.ros);

    useEffect(() => {
        let imageSubscriber: ROSLIB.Topic | null = null;

        if (isPlaying) {
            imageSubscriber = new ROSLIB.Topic({
                ros: ros,
                name: "/raspicam/compressed",
                messageType: "sensor_msgs/msg/CompressedImage",
            });

            imageSubscriber.subscribe((message: any) => {
                const base64Image = `data:image/jpeg;base64,${message.data}`;
                setImageData(base64Image);

                if (timeoutRef.current) {
                    clearTimeout(timeoutRef.current);
                }

                timeoutRef.current = setTimeout(() => {
                    setImageData(null);
                }, 2000);
            });
        }

        return () => {
            if (imageSubscriber) {
                imageSubscriber.unsubscribe();
            }
            if (timeoutRef.current) {
                clearTimeout(timeoutRef.current);
            }
        };
    }, [isPlaying, ros]);

    return (
        <div className="container" style={{ position: "relative" }}>
            {!isFullscreen && (
                <CameraFullScreenButton />
            )}
            {(isPlaying) ? (
                imageData ? (
                    <div
                        className="camera-container"
                        onMouseEnter={() => setIsHovered(true)}
                        onMouseLeave={() => setIsHovered(false)}
                    >
                        <img className="webcam-feed" src={imageData} style={{ borderRadius: dataGridStyles.borderRadius, }} alt="Webcam Feed" />
                        {isHovered && (
                            <IconButton
                                onClick={() => setIsPlaying(false)}
                                aria-label="Pause"
                                className="pause-button"
                                sx={{ position: "absolute", top: "50%", left: "50%", zIndex: 2, transform: "translate(-50%, -50%)" }}
                            >
                                <PauseCircleIcon sx={{ width: "3.5rem", height: "3.5rem" }} />
                            </IconButton>
                        )}
                    </div>
                ) : (
                    <Box sx={{ display: "flex", justifyContent: "center", alignItems: "center", height: "100%" }}>
                        <CircularProgress />
                    </Box>
                )
            ) : (
                <Box sx={{ display: "flex", justifyContent: "center", alignItems: "center", height: "100%" }}>
                    <IconButton onClick={() => setIsPlaying(true)} aria-label="Play">
                        <PlayCircleIcon sx={{ width: "3.5rem", height: "3.5rem" }} />
                    </IconButton>
                </Box>
            )}
        </div>
    );
};

export default CameraContainer;
