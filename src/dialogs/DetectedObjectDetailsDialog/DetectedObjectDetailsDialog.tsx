import React, { useEffect, useState } from 'react';
import {
    Dialog, DialogTitle, DialogContent, DialogActions, Button,
    Chip, IconButton, Box, ToggleButtonGroup, ToggleButton
} from '@mui/material';
import Typography from '@mui/material/Typography';
import Fade from '@mui/material/Fade';
import CloseIcon from '@mui/icons-material/Close';
import ViewCarouselIcon from '@mui/icons-material/ViewCarousel';
import GridViewIcon from '@mui/icons-material/GridView';
import ViewColumnIcon from '@mui/icons-material/ViewColumn';
import { useDispatch, useSelector } from 'react-redux';
import ImageContainer from './ObjectDialog/ImageContainer';
import DetectionDetails from './ObjectDialog/DetectionDetails';
import { generatePath } from './ObjectDialog/pathService';
import { RootState } from '../../store/mainStore';
import { useRos } from '../../utils/RosContext';
import { addGeneratedPath, resetClickedNotificationObject } from '../../store/reducers/applicationReducer';

const DetectedObjectDetailsDialog: React.FC = () => {
    const [openImage, setOpenImage] = useState<boolean>(false);
    const [isLoading, setIsLoading] = useState<boolean>(false);
    const [pathImage, setPathImage] = useState<string>('');
    const [serviceError, setServiceError] = useState<string>('');
    const [viewMode, setViewMode] = useState<'side-by-side' | 'object' | 'path'>('object');

    const clickedNotificationObject = useSelector((state: RootState) => state.app.clickedNotificationObject);
    const objectIdOdomMap = useSelector((state: RootState) => state.app.objectIdOdomMap);
    const dispatch = useDispatch();
    const { ros } = useRos();

    useEffect(() => {
        if (clickedNotificationObject.id !== "") {
            setOpenImage(true);
            setPathImage('');
            setServiceError('');
            setViewMode('object');
        }
    }, [clickedNotificationObject]);

    const handleClose = () => {
        dispatch(resetClickedNotificationObject());
        setOpenImage(false);
        setPathImage('');
        setServiceError('');
        setIsLoading(false);
    };

    const handleViewModeChange = (event: React.MouseEvent<HTMLElement>, newMode: 'side-by-side' | 'object' | 'path') => {
        if (newMode !== null) {
            setViewMode(newMode);
        }
    };

    const handleGeneratePath = () => {
        if (!ros || !clickedNotificationObject.id || !objectIdOdomMap || !objectIdOdomMap[clickedNotificationObject.id]) {
            setServiceError('ROS connection or odometry data not available');
            return;
        }

        setIsLoading(true);
        setServiceError('');

        generatePath(
            ros,
            objectIdOdomMap[clickedNotificationObject.id],
            (imageData) => {
                setIsLoading(false);
                setPathImage(`data:image/png;base64,${imageData}`);
                setViewMode('side-by-side');
            },
            (error) => {
                setIsLoading(false);
                setServiceError(error);
            }
        );
    };

    const handleAddToReport = () => {
        const generatedPathData = {
            id: clickedNotificationObject.id,
            pathUrl: pathImage,
        }

        dispatch(addGeneratedPath(generatedPathData));
        handleClose();
    };

    return (
        <Dialog
            open={openImage}
            onClose={handleClose}
            aria-labelledby="image-display-dialog"
            TransitionComponent={Fade}
            transitionDuration={300}
            PaperProps={{
                elevation: 8,
                sx: {
                    borderRadius: '12px',
                    overflow: 'hidden',
                    maxWidth: '800px',
                    width: '100%'
                }
            }}
        >
            <DialogTitle
                id="image-display-dialog-title"
                sx={{
                    backgroundColor: (theme) => theme.palette.primary.main,
                    color: 'white',
                    py: 2,
                    display: 'flex',
                    justifyContent: 'space-between',
                    alignItems: 'center'
                }}
            >
                <Box sx={{ display: 'flex', alignItems: 'center' }}>
                    <Typography variant="h6" component="span" fontWeight="500">
                        Object Detection Alert
                    </Typography>
                    {clickedNotificationObject && (
                        <Chip
                            label={clickedNotificationObject.class}
                            sx={{
                                ml: 2,
                                backgroundColor: 'rgba(255, 255, 255, 0.2)',
                                color: 'white',
                                fontWeight: 'bold'
                            }}
                            size="small"
                        />
                    )}
                </Box>
                <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                    <ToggleButtonGroup
                        size="small"
                        value={viewMode}
                        exclusive
                        onChange={handleViewModeChange}
                        aria-label="view mode"
                        sx={{
                            backgroundColor: 'rgba(255,255,255,0.1)',
                            '& .MuiToggleButton-root': {
                                color: 'rgba(255,255,255,0.7)',
                                '&.Mui-selected': {
                                    color: 'white',
                                    backgroundColor: 'rgba(255,255,255,0.2)'
                                }
                            }
                        }}
                    >
                        <ToggleButton value="object" aria-label="object view">
                            <ViewCarouselIcon fontSize="small" />
                        </ToggleButton>
                        <ToggleButton value="path" aria-label="path view" disabled={!pathImage}>
                            <GridViewIcon fontSize="small" />
                        </ToggleButton>
                        <ToggleButton value="side-by-side" aria-label="side by side view">
                            <ViewColumnIcon fontSize="small" />
                        </ToggleButton>
                    </ToggleButtonGroup>
                    <IconButton
                        aria-label="close"
                        onClick={handleClose}
                        sx={{
                            color: 'white',
                            '&:hover': {
                                backgroundColor: 'rgba(255, 255, 255, 0.1)'
                            }
                        }}
                    >
                        <CloseIcon />
                    </IconButton>
                </Box>
            </DialogTitle>

            <DialogContent sx={{ p: 0 }}>
                {clickedNotificationObject && (
                    <>
                        <ImageContainer
                            viewMode={viewMode}
                            objectData={clickedNotificationObject}
                            pathImage={pathImage}
                            isLoading={isLoading}
                            serviceError={serviceError}
                            onGeneratePath={handleGeneratePath}
                        />

                        <DetectionDetails detectionData={clickedNotificationObject} />
                    </>
                )}
            </DialogContent>

            <DialogActions sx={{ p: 2, backgroundColor: (theme) => theme.palette.mode === 'dark' ? theme.palette.background.default : '#f5f5f5', justifyContent: 'space-between' }}>
                <Button onClick={handleClose}>
                    Close
                </Button>
                <Box sx={{ display: 'flex', gap: 1 }}>
                    {pathImage && (
                        <Button
                            variant="outlined"
                            color="primary"
                            onClick={handleAddToReport}
                        >
                            Add to Report
                        </Button>
                    )}
                    <Button
                        variant="contained"
                        color="primary"
                        onClick={handleGeneratePath}
                        disabled={isLoading || !!pathImage}
                    >
                        {pathImage ? "Path Generated" : isLoading ? "Generating..." : "Generate Path"}
                    </Button>
                </Box>
            </DialogActions>
        </Dialog>
    );
}

export default DetectedObjectDetailsDialog;