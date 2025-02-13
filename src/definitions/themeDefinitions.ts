import { Theme } from '@mui/material/styles';

export interface ThemeType {
    theme: Theme;
    mode: 'light' | 'dark';
    isAppbarOpen: boolean;
}