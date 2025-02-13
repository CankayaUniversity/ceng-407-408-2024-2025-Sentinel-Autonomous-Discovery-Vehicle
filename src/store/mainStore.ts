import { configureStore } from '@reduxjs/toolkit';
import themeReducer from './reducers/themeReducer';
import itemReducer from './reducers/itemReducer';

export const store = configureStore({
    reducer: {
        theme: themeReducer,
        items: itemReducer,
    },
});

export type AppState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;