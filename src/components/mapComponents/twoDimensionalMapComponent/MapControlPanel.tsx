import {
    Box, FormControl, InputLabel, Select, MenuItem,
    IconButton, Tooltip, SelectChangeEvent
} from "@mui/material";
import ZoomInIcon from "@mui/icons-material/ZoomIn";
import ZoomOutIcon from "@mui/icons-material/ZoomOut";
import RefreshIcon from "@mui/icons-material/Refresh";
import DownloadIcon from "@mui/icons-material/Download";
import PaletteIcon from "@mui/icons-material/Palette";
import SettingsIcon from "@mui/icons-material/Settings";
import { useState } from "react";
import { MapControlPanelProps } from "../../../definitions/twoDimensionalMapTypeDefinitions";

const MapControlPanel = ({
    mapTopic,
    availableTopics,
    onTopicChange,
    onZoomIn,
    onZoomOut,
    onFitToView,
    onOpenPaletteDialog,
    onDownloadMap
}: MapControlPanelProps) => {

    const [hovering, setHovering] = useState(false);

    const handleMouseEnter = () => {
        setHovering(true);
    };

    const handleMouseLeave = () => {
        setHovering(false);
    };

    const handleTopicChange = (event: SelectChangeEvent) => {
        onTopicChange(event.target.value);
    };

    return (
        <Box
            sx={{ position: 'absolute', top: 20, left: 20, zIndex: 10, display: 'flex', alignItems: 'center', gap: 1 }}
            onMouseEnter={handleMouseEnter}
            onMouseLeave={handleMouseLeave}
        >
            <Tooltip title="Open Controls">
                <IconButton
                    sx={{
                        backgroundColor: 'rgba(35, 35, 35, 0.8)',
                        color: 'white',
                        '&:hover': {
                            backgroundColor: 'rgba(55, 55, 55, 0.9)',
                        },
                        padding: 2
                    }}
                >
                    <SettingsIcon />
                </IconButton>
            </Tooltip>

            {hovering && (
                <Box
                    sx={{
                        display: 'flex',
                        gap: 1,
                        backgroundColor: 'rgba(35, 35, 35, 0.8)',
                        borderRadius: 1,
                        backdropFilter: 'blur(5px)',
                        border: '1px solid rgba(255, 255, 255, 0.2)',
                        alignItems: 'center',
                        padding: 1
                    }}
                >

                    <FormControl size="small" sx={{ minWidth: 200, color: 'white' }}>
                        <InputLabel id="map-topic-label" sx={{ color: 'rgba(255, 255, 255, 0.7)' }}>
                            Map Topic
                        </InputLabel>
                        <Select
                            labelId="map-topic-label"
                            value={mapTopic}
                            label="Map Topic"
                            onChange={handleTopicChange}
                            sx={{
                                color: 'white',
                                '& .MuiOutlinedInput-notchedOutline': {
                                    borderColor: 'rgba(255, 255, 255, 0.3)'
                                },
                                '&:hover .MuiOutlinedInput-notchedOutline': {
                                    borderColor: 'rgba(255, 255, 255, 0.5)'
                                },
                                '&.Mui-focused .MuiOutlinedInput-notchedOutline': {
                                    borderColor: 'rgba(255, 255, 255, 0.7)'
                                },
                                '& .MuiSvgIcon-root': {
                                    color: 'white'
                                }
                            }}
                        >
                            {availableTopics.map((topic) => (
                                <MenuItem key={topic} value={topic}>
                                    {topic}
                                </MenuItem>
                            ))}
                        </Select>
                    </FormControl>

                    <Box sx={{ display: 'flex', gap: 1 }}>
                        <Tooltip title="Zoom In">
                            <IconButton size="small" onClick={onZoomIn} sx={{ color: 'white' }}>
                                <ZoomInIcon />
                            </IconButton>
                        </Tooltip>
                        <Tooltip title="Zoom Out">
                            <IconButton size="small" onClick={onZoomOut} sx={{ color: 'white' }}>
                                <ZoomOutIcon />
                            </IconButton>
                        </Tooltip>
                        <Tooltip title="Fit to View">
                            <IconButton size="small" onClick={onFitToView} sx={{ color: 'white' }}>
                                <RefreshIcon />
                            </IconButton>
                        </Tooltip>
                        <Tooltip title="Change Color Palette">
                            <IconButton size="small" onClick={onOpenPaletteDialog} sx={{ color: 'white' }}>
                                <PaletteIcon />
                            </IconButton>
                        </Tooltip>
                        <Tooltip title="Download Map">
                            <IconButton size="small" onClick={onDownloadMap} sx={{ color: 'white' }}>
                                <DownloadIcon />
                            </IconButton>
                        </Tooltip>
                    </Box>
                </Box>
            )}
        </Box>
    );
};

export default MapControlPanel;
