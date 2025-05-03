import React from 'react'
import { useDispatch, useSelector } from 'react-redux';
import { RootState } from '../../store/mainStore';
import { setIsDetectFrameEnabled } from '../../store/reducers/applicationReducer';
import { IconButton, Tooltip } from '@mui/material';
import CheckBoxIcon from '@mui/icons-material/CheckBox';
import CheckBoxOutlineBlankIcon from '@mui/icons-material/CheckBoxOutlineBlank';

const DetectionFrameEnableButton: React.FC = () => {
    const dispatch = useDispatch();
    const isDetectFrameEnabled = useSelector((state: RootState) => state.app.isDetectFrameEnabled);

    const toggleDetectionFrame = () => {
        dispatch(setIsDetectFrameEnabled(!isDetectFrameEnabled));
    };

    return (
        <Tooltip title={isDetectFrameEnabled ? "Disable Object Detection Frame" : "Enable Object Detection Frame"}>
            <IconButton
                aria-label="toggle detection frame"
                sx={{
                    position: "absolute",
                    top: "1rem",
                    left: "1rem",
                    zIndex: 3,
                    bgcolor: isDetectFrameEnabled ? 'rgba(0, 150, 136, 0.12)' : 'transparent',
                    '&:hover': {
                        bgcolor: isDetectFrameEnabled ? 'rgba(0, 150, 136, 0.25)' : 'rgba(0, 0, 0, 0.04)'
                    }
                }}
                onClick={toggleDetectionFrame}
            >
                {isDetectFrameEnabled ?
                    <CheckBoxIcon color="success" /> :
                    <CheckBoxOutlineBlankIcon color="action" />
                }
            </IconButton>
        </Tooltip>
    )
}

export default DetectionFrameEnableButton