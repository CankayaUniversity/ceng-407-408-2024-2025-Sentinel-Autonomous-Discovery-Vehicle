import { Box } from "@mui/material";
import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import { useRos } from "../../../utils/RosContext";
import MapControlPanel from "./MapControlPanel";
import { CircularProgress, FormControl, InputLabel, MenuItem, Select } from "@mui/material";
import MapInfoPanel from "./MapInfoPanel";
import ColorPaletteDialog from "./ColorPaletteDialog";
import { ColorPaletteKey } from "../../../definitions/twoDimensionalMapTypeDefinitions";
import { colorPalettes } from "../../../constants/mapPaletteConstants";


const TwoDimensionalMapComponent = () => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const containerRef = useRef<HTMLDivElement | null>(null);
  const [hasReceivedMap, setHasReceivedMap] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [zoomLevel, setZoomLevel] = useState(1);
  const [mapTopic, setMapTopic] = useState("/map");
  const [mapData, setMapData] = useState<any>(null);
  const [pan, setPan] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const [availableTopics, setAvailableTopics] = useState<string[]>([
    "/map",
    "/local_costmap/costmap",
    "/global_costmap/costmap",
    "/grid_prob_map",
  ]);
  const [selectedPalette, setSelectedPalette] = useState<ColorPaletteKey>("default");
  const [paletteDialogOpen, setPaletteDialogOpen] = useState(false);
  const { ros } = useRos();

  const [infoPanelVisibility, setInfoPanelVisibility] = useState<string>("hidden");

  useEffect(() => {
    if (!ros) return;

    ros.getTopics(
      (topics: any) => {
        const topicTypes = topics.types;
        const topicNames = topics.topics;

        const mapTopics = topicNames.filter((topic: string, index: number) => {
          const type = topicTypes[index];
          return type === "nav_msgs/msg/OccupancyGrid" ||
            type === "nav_msgs/OccupancyGrid";
        });

        if (mapTopics.length > 0) {
          setAvailableTopics(mapTopics);
        }
      },
      (error: any) => {
        console.error("Error getting topics:", error);
      }
    );

  }, [ros]);

  const handleTopicChange = (newTopic: string) => {
    setMapTopic(newTopic);
    setIsLoading(true);
    setHasReceivedMap(false);
  };

  const handlePaletteChange = (palette: ColorPaletteKey) => {
    setSelectedPalette(palette);
    if (mapData) {
      renderMap(mapData);
    }
  };

  const downloadMap = () => {
    if (!canvasRef.current || !mapData) return;

    const tempCanvas = document.createElement('canvas');
    const ctx = tempCanvas.getContext('2d');

    if (!ctx) return;

    tempCanvas.width = mapData.info.width;
    tempCanvas.height = mapData.info.height;

    ctx.drawImage(canvasRef.current, 0, 0);

    const link = document.createElement('a');

    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    const topicName = mapTopic.replace(/\//g, '_').substring(1);

    link.download = `ros_map_${topicName}_${timestamp}.png`;
    link.href = tempCanvas.toDataURL('image/png');
    link.click();
  };

  const renderMap = (message: any) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const { width, height } = message.info;
    const data = message.data;

    canvas.width = width;
    canvas.height = height;

    const imageData = ctx.createImageData(width, height);

    const palette = colorPalettes[selectedPalette];

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const index = y * width + x;
        const pixelIndex = index * 4;

        if (index >= 0 && index < data.length) {
          const value = data[index];
          let color;

          if (value === 100) {
            color = palette.occupied;
          } else if (value === 0) {
            color = palette.free;
          } else if (value === -1) {
            color = palette.unknown;
          } else {
            color = palette.gradient(value);
          }

          imageData.data[pixelIndex] = color.r;
          imageData.data[pixelIndex + 1] = color.g;
          imageData.data[pixelIndex + 2] = color.b;
          imageData.data[pixelIndex + 3] = 255;
        }
      }
    }

    ctx.putImageData(imageData, 0, 0);

  };


  const fitMapToView = () => {
    if (!mapData || !containerRef.current) return;

    const container = containerRef.current;
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;

    const mapWidth = mapData.info.width;
    const mapHeight = mapData.info.height;

    const widthRatio = containerWidth / mapWidth;
    const heightRatio = containerHeight / mapHeight;

    const fitZoom = Math.min(widthRatio, heightRatio) * 0.9;

    setZoomLevel(fitZoom);
    setPan({ x: 0, y: 0 });
  };

  useEffect(() => {
    if (!ros) return;

    console.log(`Subscribing to ${mapTopic}`);

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: mapTopic,
      messageType: "nav_msgs/msg/OccupancyGrid",
    });

    const timeoutId = setTimeout(() => {
      if (!hasReceivedMap) {
        console.log(`No data received from ${mapTopic} after 5 seconds`);
        listener.unsubscribe();
        setIsLoading(false);
      }
    }, 5000);

    listener.subscribe((message: any) => {
      console.log(`Received map data from ${mapTopic}`, {
        width: message.info.width,
        height: message.info.height,
        resolution: message.info.resolution
      });

      clearTimeout(timeoutId);
      setHasReceivedMap(true);
      setIsLoading(false);
      setMapData(message);

      renderMap(message);

      setTimeout(fitMapToView, 100);
    });

    return () => {
      clearTimeout(timeoutId);
      listener.unsubscribe();
    };
  }, [ros, mapTopic, selectedPalette]);

  useEffect(() => {
    if (mapData) {
      renderMap(mapData);
    }
  }, [zoomLevel, pan, selectedPalette]);

  const handleZoomIn = () => {
    setZoomLevel(prevZoom => Math.min(prevZoom * 1.2, 5));
  };

  const handleZoomOut = () => {
    setZoomLevel(prevZoom => Math.max(prevZoom / 1.2, 0.2));
  };

  const handleMouseDown = (e: React.MouseEvent) => {
    setIsDragging(true);
    setDragStart({ x: e.clientX, y: e.clientY });
  };

  const handleMouseMove = (e: React.MouseEvent) => {
    if (!isDragging) return;

    const dx = e.clientX - dragStart.x;
    const dy = e.clientY - dragStart.y;

    setPan(prevPan => ({
      x: prevPan.x + dx,
      y: prevPan.y + dy
    }));

    setDragStart({ x: e.clientX, y: e.clientY });
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  return (
    <Box
      sx={{
        width: "100%",
        height: "100%",
        position: "relative",
        bgcolor: "#2a2a2a",
        overflow: "hidden"
      }}
      onMouseEnter={() => setInfoPanelVisibility("visible")}
      onMouseLeave={() => setInfoPanelVisibility("hidden")}
    >
      {hasReceivedMap ? (
        <Box
          ref={containerRef}
          sx={{
            position: 'relative',
            width: '100%',
            height: '100%',
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            cursor: isDragging ? 'grabbing' : 'grab'
          }}
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseUp}
        >
          <MapControlPanel
            mapTopic={mapTopic}
            availableTopics={availableTopics}
            onTopicChange={handleTopicChange}
            onZoomIn={handleZoomIn}
            onZoomOut={handleZoomOut}
            onFitToView={fitMapToView}
            onOpenPaletteDialog={() => setPaletteDialogOpen(true)}
            onDownloadMap={downloadMap}
          />

          <div
            style={{
              transform: `scale(${zoomLevel}) translate(${pan.x / zoomLevel}px, ${pan.y / zoomLevel}px)`,
              transformOrigin: 'center',
              transition: 'transform 0.1s',
              boxShadow: '0 0 20px rgba(0,0,0,0.5)'
            }}
          >
            <canvas
              ref={canvasRef}
              style={{
                display: 'block'
              }}
            />
          </div>

          {mapData && (
            <MapInfoPanel
              mapTopic={mapTopic}
              mapData={mapData}
              zoomLevel={zoomLevel}
              selectedPalette={selectedPalette}
              panelVisibility={infoPanelVisibility}
            />
          )}

          <ColorPaletteDialog
            open={paletteDialogOpen}
            onClose={() => setPaletteDialogOpen(false)}
            selectedPalette={selectedPalette}
            onPaletteChange={handlePaletteChange}
          />
        </Box>
      ) : (
        <Box
          sx={{
            display: "flex",
            flexDirection: "column",
            justifyContent: "center",
            alignItems: "center",
            height: "100%",
            color: "white"
          }}
        >
          {isLoading ? (
            <>
              <CircularProgress sx={{ color: '#4dabf5' }} />
              <Box sx={{ mt: 2 }}>
                Loading map from {mapTopic}...
              </Box>
            </>
          ) : (
            <>
              <Box sx={{ mt: 2, mb: 2, textAlign: 'center' }}>
                No data available from {mapTopic}
              </Box>
              <FormControl size="small" sx={{ minWidth: 250, color: 'white', mt: 2 }}>
                <InputLabel id="map-topic-select-label" sx={{ color: 'rgba(255, 255, 255, 0.7)' }}>
                  Select a different map topic
                </InputLabel>
                <Select
                  labelId="map-topic-select-label"
                  value={mapTopic}
                  label="Select a different map topic"
                  onChange={(e) => handleTopicChange(e.target.value)}
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
            </>
          )}
        </Box>
      )}
    </Box>
  );
};

export default TwoDimensionalMapComponent;