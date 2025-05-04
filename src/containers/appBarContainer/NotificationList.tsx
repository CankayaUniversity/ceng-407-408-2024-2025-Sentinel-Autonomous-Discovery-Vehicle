import { useState } from 'react';
import List from '@mui/material/List';
import ListItem from '@mui/material/ListItem';
import Paper from '@mui/material/Paper';
import Box from '@mui/material/Box';
import IconButton from '@mui/material/IconButton';
import InfoIcon from '@mui/icons-material/Info';
import WarningIcon from '@mui/icons-material/Warning';
import ErrorIcon from '@mui/icons-material/Error';
import CloseIcon from '@mui/icons-material/Close';
import { useDispatch } from 'react-redux';
import { removeNotification } from '../../store/reducers/applicationReducer';
import { NotificationListProps } from '../../definitions/applicationTypeDefinitions';

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

const NotificationList: React.FC<NotificationListProps> = ({ notifications }) => {
    const [hoveredNotificationId, setHoveredNotificationId] = useState<string | null>(null);
    const dispatch = useDispatch();

    const deleteNotification = (id: string, event: React.MouseEvent) => {
        event.stopPropagation();
        dispatch(removeNotification(id));
    };

    const formatTimestamp = (timestamp: string): string => {
        const date = new Date(timestamp);
        return date.toLocaleString();
    };

    // TODO When Clicked on detected object notification, image should pop up.
    const renderImageIfObjectNotification = (notificationData: string, notificationId: string) => {
        if (notificationData.includes("New object detected")) {
            console.info("CLICKED, ObjectId: ", notificationId);
        }
    }

    return (
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
                                    maxHeight: "120px",
                                    cursor: "pointer",
                                }}
                                onMouseEnter={() => setHoveredNotificationId(item.id)}
                                onMouseLeave={() => setHoveredNotificationId(null)}
                                onClick={() => renderImageIfObjectNotification(item.data, item.id)}
                            >
                                <Box sx={{ marginRight: "10px" }}>
                                    {notificationTypeStyles[item.type].icon}
                                </Box>
                                <Box sx={{ display: "flex", position: "relative", flexDirection: "column", justifyContent: "space-between", width: "calc(100% - 50px)" }}>
                                    <Box sx={{ fontSize: "14px" }}>
                                        {item.data} {item.data.includes("New object detected") && ` (${item.id})`}
                                    </Box>
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
    );
};

export default NotificationList;