import { Box, IconButton } from '@mui/material';
import React from 'react'
import SettingsOverscanOutlinedIcon from '@mui/icons-material/SettingsOverscanOutlined';
import Dialog from '@mui/material/Dialog';
import DialogContent from '@mui/material/DialogContent';
import CameraContainer from './CameraContainer';
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../../store/mainStore';
import { setIsCameraDialogOpen } from '../../store/reducers/applicationReducer';

const CameraFullScreenButton: React.FC = () => {
    const dispatch = useDispatch();
    const isCameraDialogOpen = useSelector((state: RootState) => state.app.isCameraDialogOpen);

    return (
        <>
            {
                !isCameraDialogOpen && (
                    <IconButton
                        aria-label="fullscreen"
                        sx={{ position: "absolute", top: "1rem", right: "1rem", zIndex: 3, }}
                        onClick={() => dispatch(setIsCameraDialogOpen(true))}
                    >
                        <SettingsOverscanOutlinedIcon />
                    </IconButton>
                )
            }
            <Dialog open={isCameraDialogOpen} onClose={() => dispatch(setIsCameraDialogOpen(false))} maxWidth="lg" fullWidth>
                <DialogContent>
                    <Box sx={{ width: "100%", height: "80vh" }}>
                        <CameraContainer />
                    </Box>
                </DialogContent>
            </Dialog>
        </>
    );
};

export default CameraFullScreenButton