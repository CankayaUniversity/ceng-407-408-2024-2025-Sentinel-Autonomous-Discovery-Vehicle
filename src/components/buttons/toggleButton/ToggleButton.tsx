import { motion } from 'framer-motion';
import React from 'react';
import { ToggleButtonProps } from '../../../definitions/componentTypeDefinitions';
import { RootState } from '../../../store/mainStore';
import {  useSelector } from 'react-redux';


const ToggleButton: React.FC<ToggleButtonProps> = ({ setOpen, appBarStyles }) => {
    const theme = useSelector((state: RootState) => state.theme.theme);


    return (
        <span style={{ cursor: "pointer", position: "absolute", ...appBarStyles }} onClick={() => setOpen((prev) => !prev)}>
            <svg width="23" height="23" viewBox="0 0 23 23">
                <motion.path
                    strokeWidth="3"
                    stroke={theme.palette.primary.main}
                    strokeLinecap="round"
                    variants={{
                        closed: { d: 'M 2 2.5 L 20 2.5' },
                        open: { d: 'M 3 16.5 L 17 2.5' },
                    }}
                />
                <motion.path
                    strokeWidth="3"
                    stroke={theme.palette.primary.main}
                    strokeLinecap="round"
                    d="M 2 9.423 L 20 9.423"
                    variants={{
                        closed: { opacity: 1 },
                        open: { opacity: 0 },
                    }}
                />
                <motion.path
                    strokeWidth="3"
                    stroke={theme.palette.primary.main}
                    strokeLinecap="round"
                    variants={{
                        closed: { d: 'M 2 16.346 L 20 16.346' },
                        open: { d: 'M 3 2.5 L 17 16.346' },
                    }}
                />
            </svg>
        </span>
    );
};

export default ToggleButton;