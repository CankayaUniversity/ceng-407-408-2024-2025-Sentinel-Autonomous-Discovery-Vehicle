import Box from '@mui/material/Box';
import Tab from '@mui/material/Tab';
import TabContext from '@mui/lab/TabContext';
import TabList from '@mui/lab/TabList';
import TabPanel from '@mui/lab/TabPanel';
import React, { useEffect, useRef, useState } from 'react';
import { tabLabelStyles } from '../../constants/styleConstants';
import TwoDimensionalMapComponent from '../../components/mapComponents/twoDimensionalMapComponent/TwoDimensionalMapComponent';
import ThreeDimensionalMapComponent from '../../components/mapComponents/threeDimensionalMapComponent/ThreeDimensionalMapComponent';
import LidarComponent from '../../components/lidarComponent/LidarComponent';
import IconButton from '@mui/material/IconButton';
import SettingsOverscanOutlinedIcon from '@mui/icons-material/SettingsOverscanOutlined';
import Dialog from '@mui/material/Dialog';
import DialogContent from '@mui/material/DialogContent';

const MapContainer = () => {
    const [value, setValue] = useState('1');
    const [tabListHeight, setTabListHeight] = useState(0);
    const tabListRef = useRef<HTMLDivElement | null>(null);
    const [dialogOpen, setDialogOpen] = useState(false);

    useEffect(() => {
        if (tabListRef.current) {
            setTabListHeight(tabListRef.current.offsetHeight);
        }
    }, [tabListRef]);

    const handleChange = (_event: React.SyntheticEvent, newValue: string) => {
        setValue(newValue);
    };

    const handleDialogOpen = () => {
        setDialogOpen(true);
    };

    const handleDialogClose = () => {
        setDialogOpen(false);
    };

    const getSelectedTabContent = () => {
        switch (value) {
            case '1':
                return <TwoDimensionalMapComponent />;
            case '2':
                return <ThreeDimensionalMapComponent />;
            case '3':
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
                        sx={{ position: "absolute", top: "3.5rem", right: "1rem" }}
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
                        <Tab label="3D Map" value="2" sx={{ ...tabLabelStyles }} />
                        <Tab label="Lidar Scan" value="3" sx={{ ...tabLabelStyles }} />
                    </TabList>
                </Box>
                <TabPanel
                    value="1"
                    sx={{ width: '100%', height: `calc(100% - ${tabListHeight}px)`, padding: 0 }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        <TwoDimensionalMapComponent />
                    </Box>
                </TabPanel>
                <TabPanel
                    value="2"
                    sx={{ width: '100%', height: `calc(100% - ${tabListHeight}px)`, padding: 0 }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        <ThreeDimensionalMapComponent />
                    </Box>
                </TabPanel>
                <TabPanel
                    value="3"
                    sx={{ width: '100%', height: `calc(100% - ${tabListHeight}px)`, padding: 0 }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        <LidarComponent />
                    </Box>
                </TabPanel>
            </TabContext>

            <Dialog
                open={dialogOpen}
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
