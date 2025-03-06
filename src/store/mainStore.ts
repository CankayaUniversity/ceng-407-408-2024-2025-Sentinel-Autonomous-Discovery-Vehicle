import { configureStore } from '@reduxjs/toolkit';
import themeReducer from './reducers/themeReducer';
import itemReducer from './reducers/itemReducer';
import applicationReducer from './reducers/applicationReducer';

export const store = configureStore({
    reducer: {
        theme: themeReducer,
        items: itemReducer,
        app: applicationReducer
    }
});

export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;