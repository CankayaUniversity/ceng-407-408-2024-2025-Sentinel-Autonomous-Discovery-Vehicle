export interface MapInfoPanelProps {
    mapTopic: string;
    mapData: any;
    zoomLevel: number;
    selectedPalette: ColorPaletteKey;
    panelVisibility: string;
}

export interface ColorPaletteDialogProps {
    open: boolean;
    onClose: () => void;
    selectedPalette: ColorPaletteKey;
    onPaletteChange: (palette: ColorPaletteKey) => void;
}

export interface MapControlPanelProps {
    mapTopic: string;
    availableTopics: string[];
    onTopicChange: (topic: string) => void;
    onZoomIn: () => void;
    onZoomOut: () => void;
    onFitToView: () => void;
    onOpenPaletteDialog: () => void;
    onDownloadMap: () => void;
}

export type Color = {
    r: number;
    g: number;
    b: number;
};

export type ColorPalette = {
    name: string;
    occupied: Color;
    free: Color;
    unknown: Color;
    gradient: (value: number) => Color;
};

export type ColorPaletteKey = 'default' | 'highContrast' | 'heatmap' | 'nightVision' | 'blueprint';
