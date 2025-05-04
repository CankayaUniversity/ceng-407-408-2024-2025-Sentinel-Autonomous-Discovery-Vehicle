import { useEffect, useState, useRef } from 'react';
import Box from '@mui/material/Box';
import Dialog from '@mui/material/Dialog';
import DialogTitle from '@mui/material/DialogTitle';
import DialogContent from '@mui/material/DialogContent';
import IconButton from '@mui/material/IconButton';
import CloseIcon from '@mui/icons-material/Close';
import { PDFViewer } from '@react-pdf/renderer';
import { useDispatch, useSelector } from 'react-redux';
import {
    clearGeneratedMapsFromReport,
    setGenerateReport
} from '../store/reducers/applicationReducer';
import ReportGenerator from '../containers/reportGenerator/ReportGenerator';
import { RootState } from '../store/mainStore';
import { ReportViewDialogProps } from '../definitions/reportGeneratorTypeDefinitions';

const ReportViewDialog: React.FC<ReportViewDialogProps> = ({ open, onClose, missionType, selectedObjects }) => {
    const dispatch = useDispatch();
    const [isGeneratingReport, setIsGeneratingReport] = useState<boolean>(false);
    const hasInitialized = useRef(false);
    const reportData = useSelector((state: RootState) => state.app.reportData);
    const generateReport = useSelector((state: RootState) => state.app.generateReport);

    useEffect(() => {
        if (open && !hasInitialized.current) {
            dispatch(clearGeneratedMapsFromReport());
            setIsGeneratingReport(true);
            dispatch(setGenerateReport(true));
            hasInitialized.current = true;
        } else if (!open) {
            hasInitialized.current = false;
        }
    }, [open, dispatch]);

    useEffect(() => {
        if (generateReport === false && isGeneratingReport) {
            setIsGeneratingReport(false);
        }
    }, [generateReport, isGeneratingReport]);

    return (
        <Dialog
            open={open}
            onClose={onClose}
            aria-labelledby="report-dialog-title"
            maxWidth="lg"
            fullWidth
        >
            <DialogTitle id="report-dialog-title">
                {missionType} Mission Report
                <IconButton
                    aria-label="close"
                    onClick={onClose}
                    sx={{
                        position: 'absolute',
                        right: 8,
                        top: 8,
                        color: (theme) => theme.palette.grey[500],
                    }}
                >
                    <CloseIcon />
                </IconButton>
            </DialogTitle>
            <DialogContent sx={{ height: '80vh' }}>
                {isGeneratingReport ? (
                    <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', height: '100%' }}>
                        Loading report...
                    </Box>
                ) : (
                    <PDFViewer width="100%" height="100%" style={{ border: 'none' }}>
                        <ReportGenerator
                            content={{
                                ...reportData.content,
                            }}
                            missionInformation={{
                                type: missionType,
                                objectsToBeDetected: selectedObjects,
                            }}
                        />
                    </PDFViewer>
                )}
            </DialogContent>
        </Dialog>
    );
};

export default ReportViewDialog;