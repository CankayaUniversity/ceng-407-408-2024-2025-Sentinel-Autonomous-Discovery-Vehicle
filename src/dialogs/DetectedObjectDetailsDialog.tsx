import React from 'react';
import { useEffect, useState } from 'react';
import { Dialog, DialogTitle, DialogContent, DialogActions, Button, Divider, Chip, IconButton, Box } from '@mui/material';
import Typography from '@mui/material/Typography';
import Fade from '@mui/material/Fade';
import { useSelector } from 'react-redux';
import { RootState } from '../store/mainStore';
import CloseIcon from '@mui/icons-material/Close';

const DetectedObjectDetailsDialog: React.FC = () => {

    const [openImage, setOpenImage] = useState<boolean>(false);
    const clickedNotificationObject = useSelector((state: RootState) => state.app.clickedNotificationObject);

    useEffect(() => {
        if (clickedNotificationObject.id !== "") {
            setOpenImage(true);
        }
    }, [clickedNotificationObject])

    const handleClose = () => {
        setOpenImage(false);
    }

    return (
        <>
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
                        maxWidth: '650px',
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
                </DialogTitle>
                <DialogContent sx={{ p: 0 }}>
                    {clickedNotificationObject && (
                        <>
                            <Box sx={{
                                position: 'relative',
                                backgroundColor: '#f5f5f5',
                                display: 'flex',
                                justifyContent: 'center',
                                alignItems: 'center',
                                minHeight: '300px'
                            }}>
                                <img
                                    src={clickedNotificationObject.url}
                                    alt={`Detected ${clickedNotificationObject.class}`}
                                    style={{
                                        maxWidth: '100%',
                                        maxHeight: '400px',
                                        objectFit: 'contain'
                                    }}
                                />
                            </Box>
                            <Box sx={{ p: 3 }}>
                                <Typography variant="subtitle1" fontWeight="bold" gutterBottom>
                                    Detection Details
                                </Typography>
                                <Divider sx={{ mb: 2 }} />
                                <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2 }}>
                                    <Box sx={{ minWidth: '180px' }}>
                                        <Typography variant="body2" color="text.secondary">
                                            Classification
                                        </Typography>
                                        <Typography variant="body1" fontWeight="500">
                                            {clickedNotificationObject.class}
                                        </Typography>
                                    </Box>
                                    <Box sx={{ minWidth: '180px' }}></Box>
                                    <Box sx={{ minWidth: '180px' }}>
                                        <Typography variant="body2" color="text.secondary">
                                            Detection ID
                                        </Typography>
                                        <Typography variant="body1" fontWeight="500"
                                            sx={{
                                                maxWidth: '180px',
                                                overflow: 'hidden',
                                                textOverflow: 'ellipsis'
                                            }}
                                        >
                                            {clickedNotificationObject.id || 'N/A'}
                                        </Typography>
                                    </Box>
                                </Box>
                            </Box>
                        </>
                    )}
                </DialogContent>
                <DialogActions sx={{ p: 2, backgroundColor: '#f5f5f5' }}>
                    <Button onClick={handleClose} sx={{ mr: 1 }}>
                        Close
                    </Button>
                    <Button
                        variant="contained"
                        color="primary"
                        onClick={handleClose}
                    >
                        Acknowledge
                    </Button>
                </DialogActions>
            </Dialog>
        </>
    );
}

export default DetectedObjectDetailsDialog