import { ColorPaletteKey, ColorPalette } from "../definitions/twoDimensionalMapTypeDefinitions";

export const colorPalettes: Record<ColorPaletteKey, ColorPalette> = {
    default: {
        name: "Default",
        occupied: { r: 0, g: 0, b: 0 },
        free: { r: 240, g: 248, b: 255 },
        unknown: { r: 200, g: 200, b: 210 },
        gradient: (value: number) => {
            const intensity = Math.max(0, Math.min(255, 255 - (value * 2.55)));
            return { r: intensity, g: intensity + 20, b: intensity + 40 };
        }
    },
    highContrast: {
        name: "High Contrast",
        occupied: { r: 0, g: 0, b: 0 },
        free: { r: 255, g: 255, b: 255 },
        unknown: { r: 128, g: 128, b: 128 },
        gradient: (value: number) => {
            const intensity = Math.max(0, Math.min(255, 255 - (value * 2.55)));
            return { r: intensity, g: intensity, b: intensity };
        }
    },
    nightVision: {
        name: "Night Vision",
        occupied: { r: 0, g: 0, b: 0 },
        free: { r: 0, g: 255, b: 0 },
        unknown: { r: 0, g: 100, b: 0 },
        gradient: (value: number) => {
            const intensity = Math.max(0, Math.min(255, 255 - (value * 2.55)));
            return { r: 0, g: intensity, b: 0 };
        }
    },
    heatmap: {
        name: "Heatmap",
        occupied: { r: 255, g: 254, b: 0 },
        free: { r: 255, g: 0, b: 0 },
        unknown: { r: 0, g: 0, b: 255 },
        gradient: (value: number) => {
            if (value < 33) {
                return { r: 0, g: 255 - (value * 7.7), b: 255 };
            } else if (value < 66) {
                const adjustedValue = value - 33;
                return { r: adjustedValue * 7.7, g: 0, b: 255 - (adjustedValue * 7.7) };
            } else {
                const adjustedValue = value - 66;
                return { r: 255, g: adjustedValue * 7.7, b: 0 };
            }
        }
    },
    blueprint: {
        name: "Blueprint",
        occupied: { r: 0, g: 20, b: 80 },
        free: { r: 200, g: 220, b: 255 },
        unknown: { r: 100, g: 120, b: 180 },
        gradient: (value: number) => {
            const intensity = Math.max(0, Math.min(255, 255 - (value * 2.55)));
            return { r: intensity * 0.5, g: intensity * 0.7, b: intensity };
        }
    }
};
