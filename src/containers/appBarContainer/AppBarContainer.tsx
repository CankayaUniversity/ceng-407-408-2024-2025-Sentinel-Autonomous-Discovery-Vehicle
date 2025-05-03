import { useEffect, useState } from 'react';
import Box from '@mui/material/Box';
import Drawer from '@mui/material/Drawer';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemButton from '@mui/material/ListItemButton';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import Button from '@mui/material/Button';
import { motion } from 'framer-motion';
import NotificationsIcon from '@mui/icons-material/Notifications';
import { useDispatch, useSelector } from 'react-redux';
import {
    setIsAppBarOpen,
    addNotification
} from '../../store/reducers/applicationReducer';
import { useRos } from '../../utils/RosContext';
import ROSLIB from "roslib";
import "./AppBarContainer.css";
import { openAppBarStyles } from '../../constants/styleConstants';
import ToggleButton from '../../components/buttons/toggleButton/ToggleButton';
import NotificationList from './NotificationList';
import MissionTypeDialog from '../../dialogs/MissionTypeDialog';
import ReportViewDialog from '../../dialogs/ReportViewDialog';
import { RootState } from '../../store/mainStore';

const AppBarContainer: React.FC = () => {
    const open = useSelector((state: RootState) => state.app.isAppBarOpen);
    const notifications = useSelector((state: RootState) => state.app.notifications);
    const isAppBarOpen = useSelector((state: RootState) => state.app.isAppBarOpen);
    const dispatch = useDispatch();
    const { ros } = useRos();

    const [isMissionTypeDialogOpen, setIsMissionTypeDialogOpen] = useState(false);
    const [openReportDialog, setOpenReportDialog] = useState(false);
    const [missionType, setMissionType] = useState('');
    const [objectClasses, setObjectClasses] = useState(["Table", "Person", "Bag"]);
    const [selectedObjects, setSelectedObjects] = useState<string[]>([]);

    useEffect(() => {
        if (!ros) return;
        const notificationsTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/notifications',
            messageType: 'std_msgs/String'
        });

        const handleNotification = (message: any) => {
            try {
                console.info(message.data);
                const notification = JSON.parse(message.data);
                dispatch(addNotification(notification));
            } catch (error) {
                console.error('Error parsing notification message:', error);
            }
        };

        notificationsTopic.subscribe(handleNotification);

        return () => {
            notificationsTopic.unsubscribe();
        };
    }, [ros, dispatch]);

    const toggleDrawer = (newOpen: boolean) => () => {
        dispatch(setIsAppBarOpen(newOpen));
    };

    const handleGenerateReportClick = () => {
        setIsMissionTypeDialogOpen(true);
    };

    const handleMissionTypeDialogClose = () => {
        setIsMissionTypeDialogOpen(false);
    };

    const handleMissionTypeConfirm = (type: string, objects: string[]) => {
        setMissionType(type);
        setSelectedObjects(objects);
        setIsMissionTypeDialogOpen(false);
        setOpenReportDialog(true);
    };

    const handleCloseReportDialog = () => {
        setOpenReportDialog(false);
    };

    const handleToggle = () => {
        dispatch(setIsAppBarOpen(!isAppBarOpen));
    };

    const DrawerList = (
        <Box sx={{
            width: 270,
            height: "100vh",
            display: "flex",
            flexDirection: "column"
        }}>
            <Box>
                <List>
                    <ListItem disablePadding>
                        <ListItemIcon>
                            <motion.div className="hamburgerMenu" animate={open ? 'open' : 'closed'}>
                                <ToggleButton setOpen={handleToggle} appBarStyles={openAppBarStyles} />
                            </motion.div>
                        </ListItemIcon>
                    </ListItem>
                    <ListItem disablePadding>
                        <ListItemButton sx={{ "&:hover": { backgroundColor: "transparent" }, cursor: "default" }}>
                            <ListItemIcon>
                                <NotificationsIcon />
                            </ListItemIcon>
                            <ListItemText primary={"Notifications"} />
                        </ListItemButton>
                    </ListItem>
                </List>
            </Box>
            <Box sx={{
                flexGrow: 1,
                overflow: "auto",
                maxHeight: "calc(100vh - 160px)"
            }}>
                <NotificationList notifications={notifications} />
            </Box>
            <Box sx={{
                padding: "16px",
                borderTop: "1px solid rgba(0, 0, 0, 0.12)",
                backgroundColor: "background.paper",
                display: "flex",
                justifyContent: "center",
                minHeight: "80px",
                alignItems: "center"
            }}>
                <Button
                    color="secondary"
                    sx={{ height: "2.5rem" }}
                    variant="contained"
                    onClick={handleGenerateReportClick}
                >
                    Generate Report
                </Button>
            </Box>
        </Box>
    );

    return (
        <Box sx={{ position: "relative", }}>
            <Drawer sx={{ position: "relative", zIndex: 20, }} variant="persistent" open={open} onClose={toggleDrawer(false)}>
                {DrawerList}
            </Drawer>

            <MissionTypeDialog
                open={isMissionTypeDialogOpen}
                onClose={handleMissionTypeDialogClose}
                onConfirm={handleMissionTypeConfirm}
                objectClasses={objectClasses}
            />

            <ReportViewDialog
                open={openReportDialog}
                onClose={handleCloseReportDialog}
                missionType={missionType}
                selectedObjects={selectedObjects}
            />
        </Box>
    );
}

export default AppBarContainer;