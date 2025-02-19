import { motion } from "framer-motion";
import ToggleButton from "../toggleButton/ToggleButton";
import { RootState } from "../../../store/mainStore";
import { useDispatch, useSelector } from "react-redux";
import { setIsAppBarOpen } from "../../../store/reducers/applicationReducer";

const HamburgerMenuButton = ({ appBarStyles }: { appBarStyles: any }) => {
  const open = useSelector((state: RootState) => state.app.isAppBarOpen);

  const dispatch = useDispatch();

  return !open ? (
    <motion.div
      style={{ position: "absolute" }}
      animate={open ? "open" : "closed"}
    >
      <ToggleButton
        setOpen={(value) => dispatch(setIsAppBarOpen(value as any))}
        appBarStyles={appBarStyles}
      />
    </motion.div>
  ) : (
    <></>
  );
};

export default HamburgerMenuButton;
