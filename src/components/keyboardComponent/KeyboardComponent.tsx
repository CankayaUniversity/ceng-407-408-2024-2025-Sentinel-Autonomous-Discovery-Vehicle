import { Box, Typography } from "@mui/material";
import { useEffect, useState } from "react";

const KeyboardComponent = () => {
    const [pressedKey, setPressedKey] = useState<string | null>(null);

    useEffect(() => {
        // Add event listeners
        const handleKeyDown = (e: KeyboardEvent) => {
            if (["w", "a", "s", "d"].includes(e.key.toLowerCase())) {
                setPressedKey(e.key.toLowerCase());
            }
        };

        const handleKeyUp = (e: KeyboardEvent) => {
            if (["w", "a", "s", "d"].includes(e.key.toLowerCase())) {
                setPressedKey(null);
            }
        };

        document.addEventListener("keydown", handleKeyDown);
        document.addEventListener("keyup", handleKeyUp);

        return () => {
            document.removeEventListener("keydown", handleKeyDown);
            document.removeEventListener("keyup", handleKeyUp);
        };
    }, []);

    const getBackgroundColor = (key: string) => {
        return pressedKey === key ? "#1b4135" : "#33675d";
    };

    return (
        <Box sx={{ width: "100%", height: "100%", display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center" }}>
            <Box
                sx={{
                    width: "5.7rem",
                    height: "5.7rem",
                    display: "flex",
                    alignItems: "center",
                    justifyContent: "center",
                    backgroundColor: getBackgroundColor("w"),
                    border: "2px solid #1b4135",
                    margin: "0.1rem",
                    borderRadius: "10px",
                }}
            >
                <Typography sx={{ fontSize: "1.2rem" }}>W</Typography>
            </Box>
            <Box sx={{ display: "flex" }}>
                {["a", "s", "d"].map((key) => (
                    <Box
                        key={key}
                        sx={{
                            width: "5.7rem",
                            height: "5.7rem",
                            display: "flex",
                            alignItems: "center",
                            justifyContent: "center",
                            backgroundColor: getBackgroundColor(key),
                            border: "2px solid #1b4135",
                            margin: "0.1rem",
                            borderRadius: "10px",
                        }}
                    >
                        <Typography sx={{ fontSize: "1.2rem" }}>{key.toUpperCase()}</Typography>
                    </Box>
                ))}
            </Box>
        </Box>
    );
};

export default KeyboardComponent;
