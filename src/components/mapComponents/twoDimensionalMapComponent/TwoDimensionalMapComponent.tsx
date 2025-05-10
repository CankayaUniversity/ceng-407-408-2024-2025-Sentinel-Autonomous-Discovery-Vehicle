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
import { useDispatch, useSelector } from "react-redux";
import { RootState } from "../../../store/mainStore";
import { addNotification, setGenerateReport, addGeneratedMapToReport, setIsGeneratingMaps } from "../../../store/reducers/applicationReducer";
import { v4 as uuidv4 } from 'uuid';


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
  const dispatch = useDispatch();

  const [infoPanelVisibility, setInfoPanelVisibility] = useState<string>("hidden");
  const generateReport = useSelector((state: RootState) => state.app.generateReport);
  const isGeneratingMaps = useSelector((state: RootState) => state.app.isGeneratingMaps);
  const isFetchingObjects = useSelector((state: RootState) => state.app.isFetchingObjects);

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
        dispatch(addNotification({
          id: uuidv4(),
          data: `Error getting topics: ${error}`,
          timestamp: new Date().toISOString(),
          type: "INFO",
        }));
      }
    );

  }, [ros]);

  useEffect(() => {
    if (generateReport === true) {

      const allTopics = [
        "/map",
        "/global_costmap/costmap",
        "/global_costmap/static_layer",
        "/grid_prob_map"
      ];

      const allPalettes: ColorPaletteKey[] = [
        'default',
        'highContrast',
        'heatmap',
        'nightVision',
        'blueprint'
      ];

      let completedMaps = 0;
      const totalMaps = allTopics.length * allPalettes.length;
      dispatch(addNotification({
        id: uuidv4(),
        data: `Starting to generate ${totalMaps} maps...`,
        timestamp: new Date().toISOString(),
        type: "INFO",
      }));

      allTopics.forEach(topic => {
        const listener = new ROSLIB.Topic({
          ros: ros,
          name: topic,
          messageType: "nav_msgs/msg/OccupancyGrid",
        });

        const topicTimeout = setTimeout(() => {
          dispatch(addNotification({
            id: uuidv4(),
            data: `Topic ${topic} did not respond within the timeout period`,
            timestamp: new Date().toISOString(),
            type: "WARNING",
          }));
          listener.unsubscribe();

          completedMaps += allPalettes.length;
        }, 5000);

        listener.subscribe((message: any) => {
          clearTimeout(topicTimeout);

          allPalettes.forEach(palette => {
            const tempCanvas = document.createElement('canvas');
            const ctx = tempCanvas.getContext('2d');

            if (!ctx) {
              completedMaps++;
              return;
            }

            const { width, height } = message.info;
            tempCanvas.width = width;
            tempCanvas.height = height;

            const imageData = ctx.createImageData(width, height);
            const paletteColors = colorPalettes[palette];

            for (let y = 0; y < height; y++) {
              for (let x = 0; x < width; x++) {
                const index = y * width + x;
                const pixelIndex = index * 4;

                if (index >= 0 && index < message.data.length) {
                  const value = message.data[index];
                  let color;

                  if (value === 100) {
                    color = paletteColors.occupied;
                  } else if (value === 0) {
                    color = paletteColors.free;
                  } else if (value === -1) {
                    color = paletteColors.unknown;
                  } else {
                    color = paletteColors.gradient(value);
                  }

                  imageData.data[pixelIndex] = color.r;
                  imageData.data[pixelIndex + 1] = color.g;
                  imageData.data[pixelIndex + 2] = color.b;
                  imageData.data[pixelIndex + 3] = 255;
                }
              }
            }

            ctx.putImageData(imageData, 0, 0);

            const timestamp = new Date().toISOString();
            const dataUrl = tempCanvas.toDataURL('image/png');

            const serializedMapData = {
              topic: topic,
              palette: palette,
              dataUrl: dataUrl,
              timestamp: timestamp
            };

            dispatch(addGeneratedMapToReport(serializedMapData));

            completedMaps++;

            if (completedMaps >= totalMaps) {
              dispatch(setIsGeneratingMaps(false));
            }
          });

          listener.unsubscribe();
        });
      });

      setTimeout(() => {
        if (completedMaps < totalMaps) {
          dispatch(addNotification({
            id: uuidv4(),
            data: `Timed out waiting for maps. Only generated ${completedMaps}/${totalMaps}`,
            timestamp: new Date().toISOString(),
            type: "WARNING",
          }));
          dispatch(setIsGeneratingMaps(false));
        }
      }, 30000);
    }
  }, [generateReport, ros]);

  useEffect(() => {
    console.info("isGeneratingMaps: ", isGeneratingMaps);
    console.info("isFetchingObjects: ", isFetchingObjects);
    if (!isGeneratingMaps && !isFetchingObjects) {
      dispatch(setGenerateReport(false));
    }
  }, [isGeneratingMaps, isFetchingObjects])

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

    dispatch(addNotification({
      id: uuidv4(),
      data: `Subscribing to ${mapTopic}`,
      timestamp: new Date().toISOString(),
      type: "INFO",
    }));

    const listener = new ROSLIB.Topic({
      ros: ros,
      name: mapTopic,
      messageType: "nav_msgs/msg/OccupancyGrid",
    });

    const timeoutId = setTimeout(() => {
      if (!hasReceivedMap) {
        dispatch(addNotification({
          id: uuidv4(),
          data: `No data received from ${mapTopic} after 5 seconds`,
          timestamp: new Date().toISOString(),
          type: "WARNING",
        }));
        listener.unsubscribe();
        setIsLoading(false);
      }
    }, 5000);


    listener.subscribe((message: any) => {
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