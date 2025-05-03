import { useEffect, useState, useRef } from 'react';
import Box from '@mui/material/Box';
import Dialog from '@mui/material/Dialog';
import DialogTitle from '@mui/material/DialogTitle';
import DialogContent from '@mui/material/DialogContent';
import IconButton from '@mui/material/IconButton';
import CloseIcon from '@mui/icons-material/Close';
import { PDFViewer } from '@react-pdf/renderer';
import ROSLIB from "roslib";
import { v4 as uuidv4 } from 'uuid';
import { useRos } from '../utils/RosContext';
import { useDispatch, useSelector } from 'react-redux';
import {
    clearGeneratedMaps,
    setGenerateReport,
    addNotification
} from '../store/reducers/applicationReducer';
import ReportGenerator from '../containers/reportGenerator/ReportGenerator';
import { reportTemplateData } from '../containers/reportGenerator/ReportTemplate';
import { RootState } from '../store/mainStore';
import { ReportViewDialogProps } from '../definitions/reportGeneratorTypeDefinitions';

const ReportViewDialog: React.FC<ReportViewDialogProps> = ({ open, onClose, missionType, selectedObjects }) => {
    const [isGeneratingReport, setIsGeneratingReport] = useState<boolean>(false);
    const [reportData, setReportData] = useState(reportTemplateData);
    const hasInitialized = useRef(false);

    const dispatch = useDispatch();
    const { ros } = useRos();
    const generateReport = useSelector((state: RootState) => state.app.generateReport);
    const generatedMaps = useSelector((state: RootState) => state.app.generatedMaps);

    useEffect(() => {
        if (open && !hasInitialized.current) {
            setIsGeneratingReport(true);
            dispatch(setGenerateReport(true));
            hasInitialized.current = true;
        } else if (!open) {
            hasInitialized.current = false;
        }
    }, [open, dispatch]);

    useEffect(() => {
        if (generateReport === false && isGeneratingReport) {
            setReportData(prevData => ({
                ...prevData,
                content: {
                    ...prevData.content,
                    imageSections: prevData.content.imageSections.map(section =>
                        section.title === 'Generated Maps'
                            ? {
                                ...section,
                                images: generatedMaps,
                            }
                            : section
                    )
                }
            }));

            fetchObjectData();

            setIsGeneratingReport(false);
            dispatch(clearGeneratedMaps());
        }
    }, [generateReport, isGeneratingReport, dispatch, ros, generatedMaps]);

    const fetchObjectData = () => {
        if (!ros) return;

        const detectedObjectsTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/get_object_request',
            messageType: 'std_msgs/String'
        });

        const message = new ROSLIB.Message({
            data: 'get_detected_objects'
        });

        detectedObjectsTopic.publish(message);

        const responseTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/get_object_links',
            messageType: 'std_msgs/String'
        });

        responseTopic.subscribe((responseTopicMessage: ROSLIB.Message) => {
            try {
                const objectLinks = JSON.parse((responseTopicMessage as any).data);

                dispatch(addNotification({
                    id: uuidv4(),
                    data: `Object Links Received from /get_detected_objects`,
                    timestamp: new Date().toISOString(),
                    type: "INFO",
                }));

                setReportData(prevData => ({
                    ...prevData,
                    content: {
                        ...prevData.content,
                        imageSections: prevData.content.imageSections.map(section =>
                            section.title === 'Summary of Detected Object Types'
                                ? {
                                    ...section,
                                    images: objectLinks,
                                }
                                : section
                        )
                    }
                }));

                responseTopic.unsubscribe();
            } catch (error) {
                dispatch(addNotification({
                    id: uuidv4(),
                    data: `Failed to parse object links JSON: ${error}`,
                    timestamp: new Date().toISOString(),
                    type: "ERROR",
                }));
            }
        });
    };

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