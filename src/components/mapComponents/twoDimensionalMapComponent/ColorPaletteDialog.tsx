import {
    Dialog, DialogTitle, DialogContent, DialogActions,
    Button, Grid, Box
} from "@mui/material";
import { ColorPaletteDialogProps, ColorPaletteKey } from "../../../definitions/twoDimensionalMapTypeDefinitions";
import { colorPalettes } from "../../../constants/mapPaletteConstants";

const ColorPaletteDialog = ({
    open,
    onClose,
    selectedPalette,
    onPaletteChange
}: ColorPaletteDialogProps) => {
    return (
        <Dialog
            open={open}
            onClose={onClose}
            PaperProps={{
                sx: {
                    bgcolor: '#333',
                    color: 'white',
                    borderRadius: 2,
                    maxWidth: '500px'
                }
            }}
        >
            <DialogTitle sx={{ borderBottom: '1px solid rgba(255,255,255,0.1)' }}>
                Select Color Palette
            </DialogTitle>
            <DialogContent>
                <Grid container spacing={2} sx={{ mt: 1 }}>
                    {Object.entries(colorPalettes).map(([key, palette]) => (
                        <Grid item xs={6} key={key}>
                            <Button
                                variant={selectedPalette === key ? "contained" : "outlined"}
                                onClick={() => onPaletteChange(key as ColorPaletteKey)}
                                sx={{
                                    width: '100%',
                                    height: '80px',
                                    display: 'flex',
                                    flexDirection: 'column',
                                    justifyContent: 'center',
                                    position: 'relative',
                                    border: selectedPalette === key
                                        ? '2px solid #4dabf5'
                                        : '1px solid rgba(255,255,255,0.3)',
                                    bgcolor: selectedPalette === key ? 'rgba(77, 171, 245, 0.2)' : 'transparent',
                                    '&:hover': {
                                        bgcolor: selectedPalette === key
                                            ? 'rgba(77, 171, 245, 0.3)'
                                            : 'rgba(255,255,255,0.1)'
                                    }
                                }}
                            >
                                <Box sx={{ fontSize: '0.9rem', mb: 1 }}>{palette.name}</Box>
                                <Box sx={{
                                    display: 'flex',
                                    width: '80%',
                                    height: '20px',
                                    mx: 'auto',
                                    borderRadius: '4px',
                                    overflow: 'hidden'
                                }}>
                                    <Box sx={{
                                        width: '33%',
                                        bgcolor: `rgb(${palette.occupied.r},${palette.occupied.g},${palette.occupied.b})`
                                    }} />
                                    <Box sx={{
                                        width: '34%',
                                        bgcolor: `rgb(${palette.free.r},${palette.free.g},${palette.free.b})`
                                    }} />
                                    <Box sx={{
                                        width: '33%',
                                        bgcolor: `rgb(${palette.unknown.r},${palette.unknown.g},${palette.unknown.b})`
                                    }} />
                                </Box>
                            </Button>
                        </Grid>
                    ))}
                </Grid>
            </DialogContent>
            <DialogActions sx={{ borderTop: '1px solid rgba(255,255,255,0.1)', padding: 2 }}>
                <Button
                    onClick={onClose}
                    variant="contained"
                    sx={{ bgcolor: '#4dabf5', '&:hover': { bgcolor: '#2196f3' } }}
                >
                    Close
                </Button>
            </DialogActions>
        </Dialog>
    );
};

export default ColorPaletteDialog;