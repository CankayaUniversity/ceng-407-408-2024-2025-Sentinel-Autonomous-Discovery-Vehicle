import { motion } from 'framer-motion';
import ToggleButton from '../toggleButton/ToggleButton';
import { useDispatch, useSelector } from 'react-redux';
import { setIsAppbarOpen } from '../../../store/reducers/themeReducer';
import { AppState } from '../../../store/mainStore';

const HamburgerMenuButton = ({ appBarStyles }: { appBarStyles: any }) => {

    const open = useSelector((state: AppState) => state.theme.isAppbarOpen);

    const dispatch = useDispatch();

    return (
        (!open) ? <motion.div style={{ position: "absolute" }} animate={open ? 'open' : 'closed'}>
            <ToggleButton setOpen={(value) => dispatch(setIsAppbarOpen(value as any))} appBarStyles={appBarStyles} />
        </motion.div> : <></>
    )
}

export default HamburgerMenuButton