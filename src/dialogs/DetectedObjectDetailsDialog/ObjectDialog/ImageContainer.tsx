import React from 'react';
import { Box, Typography, Button, CircularProgress, Alert, useTheme } from '@mui/material';

interface ImageContainerProps {
    viewMode: 'side-by-side' | 'object' | 'path';
    objectData: {
        url: string;
        class: string;
    };
    pathImage: string;
    isLoading: boolean;
    serviceError: string;
    onGeneratePath: () => void;
}

const ImageContainer: React.FC<ImageContainerProps> = ({
    viewMode,
    objectData,
    pathImage,
    isLoading,
    serviceError,
    onGeneratePath
}) => {
    const theme = useTheme();

    // Side by side view (both images)
    if (viewMode === 'side-by-side') {
        return (
            <Box sx={{
                display: 'flex',
                flexDirection: { xs: 'column', md: 'row' },
                gap: 2,
                p: 2,
                backgroundColor: theme.palette.mode === 'dark' ? theme.palette.background.default : '#f5f5f5',
                position: 'relative',
                minHeight: '300px'
            }}>
                {/* Left side - Object Image */}
                <Box sx={{
                    flex: 1,
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    backgroundColor: theme.palette.mode === 'dark' ? theme.palette.background.paper : '#fff',
                    p: 1,
                    borderRadius: 1,
                    position: 'relative'
                }}>
                    <Typography variant="subtitle2" gutterBottom>Detected Object</Typography>
                    <Box sx={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', width: '100%' }}>
                        <img
                            src={objectData.url}
                            alt={`Detected ${objectData.class}`}
                            style={{
                                maxWidth: '100%',
                                maxHeight: '300px',
                                objectFit: 'contain',
                                ...(theme.palette.mode === 'dark' && {
                                    filter: 'brightness(0.9)'
                                })
                            }}
                        />
                    </Box>
                </Box>

                {/* Right side - Path Image */}
                <Box sx={{
                    flex: 1,
                    display: 'flex',
                    flexDirection: 'column',
                    alignItems: 'center',
                    backgroundColor: theme.palette.mode === 'dark' ? theme.palette.background.paper : '#fff',
                    p: 1,
                    borderRadius: 1,
                    position: 'relative'
                }}>
                    <Typography variant="subtitle2" gutterBottom>Generated Path</Typography>
                    <Box sx={{ flex: 1, display: 'flex', alignItems: 'center', justifyContent: 'center', width: '100%' }}>
                        {pathImage ? (
                            <img
                                src={pathImage}
                                alt="Path to object"
                                style={{
                                    maxWidth: '100%',
                                    maxHeight: '300px',
                                    objectFit: 'contain'
                                }}
                            />
                        ) : (
                            <Box sx={{
                                height: '100%',
                                width: '100%',
                                display: 'flex',
                                flexDirection: 'column',
                                justifyContent: 'center',
                                alignItems: 'center',
                                color: 'text.secondary',
                                backgroundColor: theme.palette.mode === 'dark' ? theme.palette.background.default : '#f9f9f9',
                                borderRadius: 1,
                                p: 2
                            }}>
                                {isLoading ? (
                                    <>
                                        <CircularProgress size={40} sx={{ mb: 2 }} />
                                        <Typography variant="body2">Generating path...</Typography>
                                    </>
                                ) : (
                                    <>
                                        <Typography variant="body2" align="center">
                                            Path not generated yet.
                                        </Typography>
                                        <Button
                                            variant="outlined"
                                            size="small"
                                            onClick={onGeneratePath}
                                            sx={{ mt: 2 }}
                                            disabled={isLoading}
                                        >
                                            Generate Path
                                        </Button>
                                    </>
                                )}
                            </Box>
                        )}
                    </Box>
                </Box>

                {serviceError && (
                    <Box sx={{
                        position: 'absolute',
                        bottom: 10,
                        left: 10,
                        right: 10,
                        zIndex: 10
                    }}>
                        <Alert severity="error" variant="filled">{serviceError}</Alert>
                    </Box>
                )}
            </Box>
        );
    }

    return (
        <Box sx={{
            position: 'relative',
            backgroundColor: theme.palette.mode === 'dark' ? theme.palette.background.default : '#f5f5f5',
            display: 'flex',
            justifyContent: 'center',
            alignItems: 'center',
            minHeight: '350px',
            p: 2
        }}>
            <Box sx={{
                backgroundColor: theme.palette.mode === 'dark' ? theme.palette.background.paper : '#fff',
                p: 2,
                borderRadius: 1,
                width: '100%',
                height: '100%',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center'
            }}>
                <Typography variant="subtitle2" gutterBottom>
                    {viewMode === 'object' ? 'Detected Object' : 'Generated Path'}
                </Typography>

                {viewMode === 'object' ? (
                    <img
                        src={objectData.url}
                        alt={`Detected ${objectData.class}`}
                        style={{
                            maxWidth: '100%',
                            maxHeight: '350px',
                            objectFit: 'contain',
                            ...(theme.palette.mode === 'dark' && {
                                filter: 'brightness(0.9)'
                            })
                        }}
                    />
                ) : pathImage ? (
                    <img
                        src={pathImage}
                        alt="Path to object"
                        style={{
                            maxWidth: '100%',
                            maxHeight: '350px',
                            objectFit: 'contain',
                            ...(theme.palette.mode === 'dark' && {
                                filter: 'brightness(0.9)'
                            })
                        }}
                    />
                ) : (
                    <Box sx={{
                        display: 'flex',
                        flexDirection: 'column',
                        justifyContent: 'center',
                        alignItems: 'center',
                        height: '100%',
                        color: 'text.secondary'
                    }}>
                        {isLoading ? (
                            <>
                                <CircularProgress size={40} sx={{ mb: 2 }} />
                                <Typography variant="body2">Generating path...</Typography>
                            </>
                        ) : (
                            <>
                                <Typography variant="body1" sx={{ mb: 2 }}>
                                    Path not generated yet
                                </Typography>
                                <Button
                                    variant="contained"
                                    onClick={onGeneratePath}
                                    disabled={isLoading}
                                >
                                    Generate Path
                                </Button>
                            </>
                        )}
                    </Box>
                )}
            </Box>

            {serviceError && (
                <Box sx={{
                    position: 'absolute',
                    bottom: 10,
                    left: 10,
                    right: 10,
                    zIndex: 10
                }}>
                    <Alert severity="error" variant="filled">{serviceError}</Alert>
                </Box>
            )}
        </Box>
    );
};

export default ImageContainer;