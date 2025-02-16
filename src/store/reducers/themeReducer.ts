import { createSlice } from '@reduxjs/toolkit';
import { darkTheme, lightTheme } from '../../constants/themeConstants';
import { ThemeType } from '../../definitions/themeDefinitions';

const initialState: ThemeType = {
    theme: darkTheme,
    mode: 'dark',
    isAppbarOpen: false,
};

const themeSlice = createSlice({
    name: 'theme',
    initialState,
    reducers: {
        toggleTheme: (state) => {
            state.mode = state.mode === 'light' ? 'dark' : 'light';
            (state.theme as any) = state.mode === 'light' ? lightTheme : darkTheme;
        },
        setIsAppbarOpen: (state, action: { payload: boolean }) => {
            state.isAppbarOpen = action.payload;
        },
    },
});

export const { toggleTheme, setIsAppbarOpen } = themeSlice.actions;
export default themeSlice.reducer;