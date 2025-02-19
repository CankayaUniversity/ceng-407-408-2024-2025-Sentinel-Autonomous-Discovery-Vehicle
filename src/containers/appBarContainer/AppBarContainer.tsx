import Box from '@mui/material/Box';
import Drawer from '@mui/material/Drawer';
import List from '@mui/material/List';
import Divider from '@mui/material/Divider';
import ListItem from '@mui/material/ListItem';
import ListItemButton from '@mui/material/ListItemButton';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import { motion } from 'framer-motion';
import ToggleButton from '../../components/buttons/toggleButton/ToggleButton';
import "./AppBarContainer.css";
import { openAppBarStyles } from '../../constants/styleConstants';
import NotificationsIcon from '@mui/icons-material/Notifications';
import { RootState } from '../../store/mainStore';
import { useDispatch, useSelector } from 'react-redux';
import { setIsAppBarOpen } from '../../store/reducers/applicationReducer';


const AppBarContainer = () => {

    const open = useSelector((state: RootState) => state.app.isAppBarOpen);
    const dispatch = useDispatch();

    const toggleDrawer = (newOpen: boolean) => () => {
        dispatch(setIsAppBarOpen(newOpen));
    };

    const DrawerList = (
        <Box sx={{ width: 270 }} role="presentation" onClick={toggleDrawer(false)}>
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
                {
                    //TODO notifications are listed here
                }
            </List>
            <Divider />
        </Box>
    );

    return (
        <Box sx={{ position: "relative" }}>
            <Drawer sx={{ position: "relative", zIndex: 1, }} variant="persistent" open={open} onClose={toggleDrawer(false)}>
                {DrawerList}
            </Drawer>
        </Box>
    );
}

export default AppBarContainer