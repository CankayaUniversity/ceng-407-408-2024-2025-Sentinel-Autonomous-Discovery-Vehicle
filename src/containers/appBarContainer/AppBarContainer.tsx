import { useEffect, useState } from 'react';
import Box from '@mui/material/Box';
import Drawer from '@mui/material/Drawer';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import ListItemButton from '@mui/material/ListItemButton';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import Paper from '@mui/material/Paper';
import IconButton from '@mui/material/IconButton';
import Dialog from '@mui/material/Dialog';
import DialogTitle from '@mui/material/DialogTitle';
import DialogContent from '@mui/material/DialogContent';
import { PDFViewer } from '@react-pdf/renderer';
import { motion } from 'framer-motion';
import ToggleButton from '../../components/buttons/toggleButton/ToggleButton';
import "./AppBarContainer.css";
import { openAppBarStyles } from '../../constants/styleConstants';
import NotificationsIcon from '@mui/icons-material/Notifications';
import InfoIcon from '@mui/icons-material/Info';
import WarningIcon from '@mui/icons-material/Warning';
import ErrorIcon from '@mui/icons-material/Error';
import CloseIcon from '@mui/icons-material/Close';
import { RootState } from '../../store/mainStore';
import { useDispatch, useSelector } from 'react-redux';
import {
    clearGeneratedMaps,
    setGenerateReport,
    setIsAppBarOpen,
    removeNotification,
    addNotification
} from '../../store/reducers/applicationReducer';
import { useRos } from '../../utils/RosContext';
import ROSLIB from "roslib";
import { Button } from '@mui/material';
import ReportGenerator from '../reportGenerator/ReportGenerator';
import { reportTemplateData } from '../reportGenerator/ReportTemplate';

const AppBarContainer = () => {
    const open = useSelector((state: RootState) => state.app.isAppBarOpen);
    const notifications = useSelector((state: RootState) => state.app.notifications);
    const dispatch = useDispatch();
    const { ros } = useRos();

    const [hoveredNotificationId, setHoveredNotificationId] = useState<string | null>(null);
    const [openDialog, setOpenDialog] = useState(false);
    const [isGeneratingReport, setIsGeneratingReport] = useState(false);
    const [reportData, setReportData] = useState(reportTemplateData);
    const generateReport = useSelector((state: RootState) => state.app.generateReport);
    const generatedMaps = useSelector((state: RootState) => state.app.generatedMaps);

    const notificationTypeStyles = {
        INFO: {
            backgroundColor: 'rgba(33, 150, 243, 0.1)',
            borderLeft: '4px solid #2196f3',
            icon: <InfoIcon sx={{ color: '#2196f3' }} />
        },
        WARNING: {
            backgroundColor: 'rgba(255, 152, 0, 0.1)',
            borderLeft: '4px solid #ff9800',
            icon: <WarningIcon sx={{ color: '#ff9800' }} />
        },
        ERROR: {
            backgroundColor: 'rgba(244, 67, 54, 0.1)',
            borderLeft: '4px solid #f44336',
            icon: <ErrorIcon sx={{ color: '#f44336' }} />
        }
    };

    useEffect(() => {
        if (!ros) return;
        const notificationsTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/notifications',
            messageType: 'std_msgs/String'
        });

        const handleNotification = (message: ROSLIB.Message) => {
            try {
                const notification = JSON.parse((message as any).data);
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

    useEffect(() => {
        if (openDialog) {
            prepareReportData();
        }
    }, [openDialog, notifications]);

    const prepareReportData = () => {
        setIsGeneratingReport(true);
        dispatch(setGenerateReport(true));
    };

    useEffect(() => {
        if (generateReport === false) {
            setIsGeneratingReport(false);
            setReportData(prevData => ({
                ...prevData,
                content: {
                    ...prevData.content,
                    imageSections: prevData.content.imageSections.map(section =>
                        section.title === 'Generated Maps'
                            ? {
                                ...section,
                                images: generatedMaps,
                            }
                            : section
                    )
                }
            }));
            dispatch(clearGeneratedMaps());
        }
    }, [generateReport, dispatch])

    const toggleDrawer = (newOpen: boolean) => () => {
        dispatch(setIsAppBarOpen(newOpen));
    };

    const deleteNotification = (id: string, event: React.MouseEvent) => {
        event.stopPropagation();
        dispatch(removeNotification(id));
    };

    const formatTimestamp = (timestamp: string): string => {
        const date = new Date(timestamp);
        return date.toLocaleString();
    };

    const handleOpenDialog = () => {
        setOpenDialog(true);
    };

    const handleCloseDialog = () => {
        setOpenDialog(false);
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
                                <ToggleButton setOpen={(value) => dispatch(setIsAppBarOpen(value as any))} appBarStyles={openAppBarStyles} />
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
                <List>
                    {notifications && notifications.length > 0 ? (
                        notifications.map((item) => (
                            <div key={item.id}>
                                <ListItem onClick={(event) => event.stopPropagation()}>
                                    <Paper
                                        elevation={0}
                                        sx={{
                                            width: "100%",
                                            padding: "10px",
                                            display: "flex",
                                            alignItems: "flex-start",
                                            backgroundColor: notificationTypeStyles[item.type].backgroundColor,
                                            borderLeft: notificationTypeStyles[item.type].borderLeft,
                                            borderRadius: "4px",
                                            margin: "4px 0",
                                            position: "relative",
                                            minHeight: "80px",
                                            maxHeight: "120px"
                                        }}
                                        onMouseEnter={() => setHoveredNotificationId(item.id)}
                                        onMouseLeave={() => setHoveredNotificationId(null)}
                                    >
                                        <Box sx={{ marginRight: "10px" }}>
                                            {notificationTypeStyles[item.type].icon}
                                        </Box>
                                        <Box sx={{ display: "flex", position: "relative", flexDirection: "column", justifyContent: "space-between", width: "calc(100% - 50px)" }}>
                                            <Box sx={{ fontSize: "14px" }}>{item.data}</Box>
                                            <Box sx={{ fontSize: "12px", color: "text.secondary", marginTop: "4px", }}>
                                                {formatTimestamp(item.timestamp)}
                                            </Box>
                                        </Box>
                                        <IconButton
                                            size="small"
                                            sx={{
                                                position: "absolute",
                                                zIndex: 2,
                                                top: "4px",
                                                right: "4px",
                                                padding: "2px",
                                                visibility: hoveredNotificationId === item.id ? 'visible' : 'hidden',
                                                opacity: hoveredNotificationId === item.id ? 1 : 0,
                                                transition: 'opacity 0.2s ease-in-out, visibility 0.2s ease-in-out',
                                            }}
                                            onClick={(event) => deleteNotification(item.id, event)}
                                            aria-label="delete notification"
                                        >
                                            <CloseIcon fontSize="small" />
                                        </IconButton>
                                    </Paper>
                                </ListItem>
                            </div>
                        ))
                    ) : (
                        <ListItem sx={{ width: "100%", padding: "5px", color: "gray", justifyContent: "center" }}>
                            No notifications available.
                        </ListItem>
                    )}
                </List>
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
                    onClick={handleOpenDialog}
                >
                    Generate Report
                </Button>
            </Box>
        </Box>
    );

    return (
        <Box sx={{ position: "relative" }}>
            <Drawer sx={{ position: "relative", zIndex: 1, }} variant="persistent" open={open} onClose={toggleDrawer(false)}>
                {DrawerList}
            </Drawer>
            <Dialog
                open={openDialog}
                onClose={handleCloseDialog}
                aria-labelledby="report-dialog-title"
                maxWidth="lg"
                fullWidth
            >
                <DialogTitle id="report-dialog-title">
                    System Notification Report
                    <IconButton
                        aria-label="close"
                        onClick={handleCloseDialog}
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
                <DialogContent sx={{ height: '80vh' }}>
                    {isGeneratingReport ? (
                        <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100%' }}>
                            Loading report...
                        </Box>
                    ) : (
                        <PDFViewer width="100%" height="100%" style={{ border: 'none' }}>
                            <ReportGenerator
                                content={reportData.content}
                            />
                        </PDFViewer>
                    )}
                </DialogContent>
            </Dialog>
        </Box>
    );
}

export default AppBarContainer;