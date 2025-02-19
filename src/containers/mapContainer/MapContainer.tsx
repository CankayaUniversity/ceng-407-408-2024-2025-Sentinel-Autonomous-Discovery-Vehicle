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

const MapContainer = () => {
    const [value, setValue] = React.useState('1');
    const [tabListHeight, setTabListHeight] = useState(0);
    const tabListRef = useRef<HTMLDivElement | null>(null);

    useEffect(() => {
        if (tabListRef.current) {
            setTabListHeight(tabListRef.current.offsetHeight);
        }
    }, [tabListRef]);

    const handleChange = (_event: React.SyntheticEvent, newValue: string) => {
        setValue(newValue);
    };

    return (
        <Box sx={{ width: '100%', height: '100%', typography: 'body1' }}>
            <TabContext value={value}>
                <Box
                    ref={tabListRef}
                    sx={{ width: '100%', height: 'auto', borderBottom: 1, borderColor: 'divider' }}
                >
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
                    sx={{
                        width: '100%',
                        height: `calc(100% - ${tabListHeight}px)`,
                        padding: 0,
                    }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        <TwoDimensionalMapComponent />
                    </Box>
                </TabPanel>
                <TabPanel
                    value="2"
                    sx={{
                        width: '100%',
                        height: `calc(100% - ${tabListHeight}px)`,
                        padding: 0,
                    }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        <ThreeDimensionalMapComponent />
                    </Box>
                </TabPanel>
                <TabPanel
                    value="3"
                    sx={{
                        width: '100%',
                        height: `calc(100% - ${tabListHeight}px)`,
                        padding: 0,
                    }}
                >
                    <Box sx={{ width: '100%', height: '100%' }}>
                        <LidarComponent />
                    </Box>
                </TabPanel>
            </TabContext>
        </Box>
    );
};

export default MapContainer;
