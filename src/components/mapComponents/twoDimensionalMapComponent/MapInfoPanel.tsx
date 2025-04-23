import { Box } from "@mui/material";
import { colorPalettes } from "../../../constants/mapPaletteConstants";
import { MapInfoPanelProps } from "../../../definitions/twoDimensionalMapTypeDefinitions";

const MapInfoPanel = ({
    mapTopic,
    mapData,
    zoomLevel,
    selectedPalette,
}: MapInfoPanelProps) => {

    return (
        <Box
            sx={{
                position: 'absolute',
                bottom: 10,
                left: 10,
                backgroundColor: 'rgba(35, 35, 35, 0.8)',
                backdropFilter: 'blur(5px)',
                padding: 1.5,
                borderRadius: 1,
                color: 'white',
                fontSize: '0.75rem',
                maxWidth: '250px',
                border: '1px solid rgba(255, 255, 255, 0.2)'
            }}
        >
            <div>Topic: {mapTopic}</div>
            <div>Dimensions: {mapData.info.width} × {mapData.info.height}</div>
            <div>Zoom: {Math.round(zoomLevel * 100)}%</div>
            <div>Palette: {colorPalettes[selectedPalette].name}</div>
        </Box>
    );
};

export default MapInfoPanel;