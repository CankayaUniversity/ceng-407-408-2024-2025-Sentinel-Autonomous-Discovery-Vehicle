import Box from '@mui/material/Box';
import Tab from '@mui/material/Tab';
import TabContext from '@mui/lab/TabContext';
import TabList from '@mui/lab/TabList';
import TabPanel from '@mui/lab/TabPanel';
import React, { useEffect, useRef, useState } from 'react';
import KeyboardComponent from '../../components/keyboardComponent/KeyboardComponent';
import JoystickComponent from '../../components/joystickComponent/JoystickComponent';
import { tabLabelStyles } from '../../constants/styleConstants';

const MovementContainer = () => {

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
                    sx={{ width: '100%', height: 'auto', borderBottom: 1, borderColor: 'divider' }}
                    ref={tabListRef}
                >
                    <TabList onChange={handleChange} aria-label="movement tabs">
                        <Tab label="Joystick" sx={{ ...tabLabelStyles }} value="1" />
                        <Tab label="Keyboard" sx={{ ...tabLabelStyles }} value="2" />
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
                        <JoystickComponent />
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
                        <KeyboardComponent />
                    </Box>
                </TabPanel>
            </TabContext>
        </Box>
    )
}

export default MovementContainer