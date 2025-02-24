import { Box, IconButton } from '@mui/material';
import React, { useState } from 'react'
import SettingsOverscanOutlinedIcon from '@mui/icons-material/SettingsOverscanOutlined';
import Dialog from '@mui/material/Dialog';
import DialogContent from '@mui/material/DialogContent';
import CameraContainer from './CameraContainer';

const CameraFullScreenButton: React.FC = () => {
    const [dialogOpen, setDialogOpen] = useState(false);

    return (
        <>
            <IconButton
                aria-label="fullscreen"
                sx={{ position: "absolute", top: "1rem", right: "1rem", zIndex: 3, }}
                onClick={() => setDialogOpen(true)}
            >
                <SettingsOverscanOutlinedIcon />
            </IconButton>

            <Dialog open={dialogOpen} onClose={() => setDialogOpen(false)} maxWidth="lg" fullWidth>
                <DialogContent>
                    <Box sx={{ width: "100%", height: "80vh" }}>
                        <CameraContainer isFullscreen={true} />
                    </Box>
                </DialogContent>
            </Dialog>
        </>
    );
};

export default CameraFullScreenButton