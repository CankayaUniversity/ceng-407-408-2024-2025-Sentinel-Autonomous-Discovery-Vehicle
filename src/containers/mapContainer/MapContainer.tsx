import Box from '@mui/material/Box';
import Tab from '@mui/material/Tab';
import TabContext from '@mui/lab/TabContext';
import TabList from '@mui/lab/TabList';
import TabPanel from '@mui/lab/TabPanel';
import React, { useEffect, useRef, useState } from 'react';
import { tabLabelStyles } from '../../constants/styleConstants';
import TwoDimensionalMapComponent from '../../components/mapComponents/twoDimensionalMapComponent/TwoDimensionalMapComponent';
import LidarComponent from '../../components/lidarComponent/LidarComponent';
import IconButton from '@mui/material/IconButton';
import SettingsOverscanOutlinedIcon from '@mui/icons-material/SettingsOverscanOutlined';
import Dialog from '@mui/material/Dialog';
import DialogContent from '@mui/material/DialogContent';
import { RootState } from '../../store/mainStore';
import { useSelector, useDispatch } from 'react-redux';
import { setIsMapDialogOpen } from '../../store/reducers/applicationReducer';

const MapContainer = () => {
    const [value, setValue] = useState('1');
    const [tabListHeight, setTabListHeight] = useState(0);
    const tabListRef = useRef<HTMLDivElement | null>(null);
    const dispatch = useDispatch();
    const isMapDialogOpen = useSelector((state: RootState) => state.app.isMapDialogOpen);
    const isCameraDialogOpen = useSelector((state: RootState) => state.app.isCameraDialogOpen);

    useEffect(() => {
        if (tabListRef.current) {
            setTabListHeight(tabListRef.current.offsetHeight);
        }
    }, [tabListRef]);

    const handleChange = (_event: React.SyntheticEvent, newValue: string) => {
        setValue(newValue);
    };

    const handleDialogOpen = () => {
        dispatch(setIsMapDialogOpen(true));
    };

    const handleDialogClose = () => {
        dispatch(setIsMapDialogOpen(false));
    };

    const getSelectedTabContent = () => {
        switch (value) {
            case '1':
                return <TwoDimensionalMapComponent />;
            case '2':
                return <LidarComponent />;
            default:
                return null;
        }
    };

    return (
        <Box sx={{ width: '100%', height: '100%', typography: 'body1' }}>
            <TabContext value={value}>
                <Box
                    ref={tabListRef}
                    sx={{ width: '100%', height: 'auto', borderBottom: 1, borderColor: 'divider', position: "relative" }}
                >
                    <IconButton
                        aria-label="fullscreen"
                        sx={{ position: "absolute", top: "0.5rem", right: "0.5rem", zIndex: 2 }}
                        onClick={handleDialogOpen}
                    >
                        <SettingsOverscanOutlinedIcon />
                    </IconButton>
                    <TabList
                        onChange={handleChange}
                        aria-label="map container tabs"
                        textColor="primary"
                        indicatorColor="primary"
                    >
                        <Tab label="2D Map" value="1" sx={{ ...tabLabelStyles }} />
                        <Tab label="Lidar Scan" value="2" sx={{ ...tabLabelStyles }} />
                    </TabList>
                </Box>
                <TabPanel
                    value="1"
                    sx={{ width: '100%', height: `calc(100% - ${tabListHeight}px)`, padding: 0 }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        {
                            (!isMapDialogOpen && !isCameraDialogOpen) && (
                                <TwoDimensionalMapComponent />
                            )
                        }
                    </Box>
                </TabPanel>
                <TabPanel
                    value="2"
                    sx={{ width: '100%', height: `calc(100% - ${tabListHeight}px)`, padding: 0 }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        {
                            (!isMapDialogOpen && !isCameraDialogOpen) && (
                                <LidarComponent />
                            )
                        }
                    </Box>
                </TabPanel>
            </TabContext>

            <Dialog
                open={isMapDialogOpen}
                onClose={handleDialogClose}
                maxWidth="lg"
                fullWidth
            >
                <DialogContent>
                    <Box sx={{ width: '100%', height: '80vh' }}>
                        {getSelectedTabContent()}
                    </Box>
                </DialogContent>
            </Dialog>
        </Box>
    );
};

export default MapContainer;
