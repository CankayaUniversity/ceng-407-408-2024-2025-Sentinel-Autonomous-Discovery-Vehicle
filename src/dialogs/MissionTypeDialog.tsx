import { useEffect, useRef, useState } from 'react';
import Box from '@mui/material/Box';
import Dialog from '@mui/material/Dialog';
import DialogTitle from '@mui/material/DialogTitle';
import DialogContent from '@mui/material/DialogContent';
import DialogActions from '@mui/material/DialogActions';
import FormControl from '@mui/material/FormControl';
import InputLabel from '@mui/material/InputLabel';
import Select, { SelectChangeEvent } from '@mui/material/Select';
import MenuItem from '@mui/material/MenuItem';
import Button from '@mui/material/Button';
import IconButton from '@mui/material/IconButton';
import CloseIcon from '@mui/icons-material/Close';
import Typography from '@mui/material/Typography';
import Chip from '@mui/material/Chip';
import { MissionTypeDialogProps } from '../definitions/reportGeneratorTypeDefinitions';
import { useDispatch } from 'react-redux';
import { clearGeneratedMapsFromReport, setGenerateReport, setIsFetchingObjects, setIsGeneratingMaps } from '../store/reducers/applicationReducer';

const MissionTypeDialog: React.FC<MissionTypeDialogProps> = ({ open, onClose, onConfirm, objectClasses }) => {
    const [missionType, setMissionType] = useState<string>('');
    const [selectedObjects, setSelectedObjects] = useState<string[]>([]);
    const dispatch = useDispatch();
    const hasInitialized = useRef(false);

    const handleMissionTypeSelect = (event: SelectChangeEvent) => {
        setMissionType(event.target.value);
        setSelectedObjects([]);
    };

    const handleObjectChange = (event: SelectChangeEvent<string[]>) => {
        const {
            target: { value },
        } = event;
        setSelectedObjects(
            typeof value === 'string' ? value.split(',') : value,
        );
    };

    const handleConfirm = () => {
        onConfirm(missionType, selectedObjects);
        setMissionType('');
        setSelectedObjects([]);
    };

    const handleClose = () => {
        onClose();
        setMissionType('');
        setSelectedObjects([]);
    };

    useEffect(() => {
        if (open && !hasInitialized.current) {
            dispatch(clearGeneratedMapsFromReport());
            dispatch(setGenerateReport(true));
            dispatch(setIsFetchingObjects(true));
            dispatch(setIsGeneratingMaps(true));
            hasInitialized.current = true;
        } else if (!open) {
            hasInitialized.current = false;
        }
    }, [open, dispatch]);

    return (
        <Dialog
            open={open}
            onClose={handleClose}
            aria-labelledby="mission-type-dialog-title"
        >
            <DialogTitle id="mission-type-dialog-title">
                Select Mission Type
                <IconButton
                    aria-label="close"
                    onClick={handleClose}
                    sx={{
                        position: 'absolute',
                        right: 8,
                        top: 8,
                        color: (theme) => theme.palette.grey[500],
                    }}
                >
                    <CloseIcon />
                </IconButton>
            </DialogTitle>
            <DialogContent sx={{ minWidth: '500px', height: "180px", margin: "1rem" }}>
                <Box sx={{ mb: 3, paddingTop: "10px" }}>
                    <FormControl fullWidth>
                        <InputLabel id="mission-type-label">Mission Type</InputLabel>
                        <Select
                            labelId="mission-type-label"
                            id="mission-type-select"
                            value={missionType}
                            label="Mission Type"
                            onChange={handleMissionTypeSelect}
                        >
                            <MenuItem value="Rescue">Rescue</MenuItem>
                            <MenuItem value="Find and Detect">Find and Detect Objects</MenuItem>
                        </Select>
                    </FormControl>
                </Box>
                <Box sx={{ mb: 2 }}>
                    <FormControl fullWidth disabled={missionType !== 'Find and Detect'}>
                        <InputLabel
                            id="object-select-label"
                            sx={{
                                px: 1,
                            }}
                        >
                            Target Objects
                        </InputLabel>
                        <Select
                            labelId="object-select-label"
                            id="object-select"
                            multiple
                            value={selectedObjects}
                            label="Target Objects"
                            onChange={handleObjectChange}
                            renderValue={(selected) => (
                                <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 0.5 }}>
                                    {selected.map((value) => (
                                        <Chip key={value} label={value} size="small" />
                                    ))}
                                </Box>
                            )}
                            MenuProps={{
                                PaperProps: {
                                    style: {
                                        maxHeight: 224,
                                        width: 250,
                                    },
                                    sx: {
                                        bgcolor: 'background.paper',
                                    }
                                }
                            }}
                        >
                            {objectClasses.map((className) => (
                                <MenuItem key={className} value={className}>
                                    {className}
                                </MenuItem>
                            ))}
                        </Select>
                        {missionType === 'Find and Detect' && selectedObjects.length === 0 && (
                            <Typography variant="caption" color="text.secondary" sx={{ mt: 1 }}>
                                Please select at least one target object
                            </Typography>
                        )}
                    </FormControl>
                </Box>
            </DialogContent>
            <DialogActions>
                <Button
                    variant="outlined"
                    sx={{ width: "100px", height: "2.2rem" }}
                    onClick={handleClose}
                >
                    Cancel
                </Button>
                <Button
                    onClick={handleConfirm}
                    disabled={!missionType || (missionType === 'Find and Detect' && selectedObjects.length === 0)}
                    color="primary"
                    variant="contained"
                    sx={{ width: "100px", height: "2.2rem" }}
                >
                    Continue
                </Button>
            </DialogActions>
        </Dialog>
    );
};

export default MissionTypeDialog;