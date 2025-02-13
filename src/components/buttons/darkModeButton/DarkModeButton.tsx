import { toggleTheme } from '../../../store/reducers/themeReducer';
import Button from '@mui/material/Button';
import DarkModeIcon from '@mui/icons-material/DarkMode';
import { useDispatch, useSelector } from 'react-redux';
import { AppState } from '../../../store/mainStore';
import LightModeIcon from '@mui/icons-material/LightMode';

const DarkModeButton = () => {
    const dispatch = useDispatch();
    const theme = useSelector((state: AppState) => state.theme.theme);

    return (
        <Button sx={{ position: "absolute", top: "0.8rem", right: "0.8rem" }} variant="text" onClick={() => dispatch(toggleTheme())}>
            {theme.palette.mode === 'dark' ? <LightModeIcon /> : <DarkModeIcon />}
        </Button>
    )
}

export default DarkModeButton