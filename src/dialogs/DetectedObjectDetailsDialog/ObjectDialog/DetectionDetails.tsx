import React from 'react';
import { Box, Typography, Divider } from '@mui/material';

interface DetectionDetailsProps {
    detectionData: {
        class: string;
        id: string;
    };
}

const DetectionDetails: React.FC<DetectionDetailsProps> = ({ detectionData }) => {
    return (
        <Box sx={{ p: 3 }}>
            <Typography variant="subtitle1" fontWeight="bold" gutterBottom>
                Detection Details
            </Typography>
            <Divider sx={{ mb: 2 }} />
            <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 2, }}>
                <Box sx={{ minWidth: '180px' }}>
                    <Typography variant="body2" color="text.secondary">
                        Classification
                    </Typography>
                    <Typography variant="body1" fontWeight="500">
                        {detectionData.class}
                    </Typography>
                </Box>
                <Box sx={{ minWidth: '180px' }}></Box>
                <Box sx={{ minWidth: '180px' }}>
                    <Typography variant="body2" color="text.secondary">
                        Detection ID
                    </Typography>
                    <Typography variant="body1" fontWeight="500"
                        sx={{
                            maxWidth: '180px',
                            overflow: 'hidden',
                            textOverflow: 'ellipsis'
                        }}
                    >
                        {detectionData.id || 'N/A'}
                    </Typography>
                </Box>
            </Box>
        </Box>
    );
};

export default DetectionDetails;