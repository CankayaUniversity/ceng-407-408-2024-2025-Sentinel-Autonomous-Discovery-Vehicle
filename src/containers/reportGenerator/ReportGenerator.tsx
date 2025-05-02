import React from 'react';
import { Page, Text, View, Document, Image } from '@react-pdf/renderer';
import { reportStyles } from './ReportGeneratorStyles';
import { ReportGeneratorProps } from '../../definitions/reportGeneratorTypeDefinitions';

const ReportGenerator: React.FC<ReportGeneratorProps> = ({ content, missionInformation }) => {

    console.info("Mission Info", missionInformation); // TODO Render images based on the mission

    return (
        <Document>
            <Page size="A4" style={reportStyles.page}>
                <View style={reportStyles.header}>
                    <Text style={reportStyles.title}>{content.title}</Text>
                    <Text style={reportStyles.subtitle}>{content.subtitle}</Text>
                </View>

                {content.dataSections.map((section, index) => (
                    <View key={`data-section-${index}`} style={reportStyles.section}>
                        <Text style={reportStyles.sectionTitle}>{section.title}</Text>
                        <Text style={reportStyles.sectionContent}>{section.content}</Text>
                    </View>
                ))}

                {content.imageSections.map((section, index) => (
                    <View key={`image-section-${index}`} style={reportStyles.section}>
                        <Text style={reportStyles.sectionTitle}>{section.title}</Text>
                        <Text style={reportStyles.sectionContent}>{section.content}</Text>
                        <View style={reportStyles.imageGrid}>
                            {section.images.map((img, imgIndex) => {
                                const src = typeof img === 'string' ? img : img.dataUrl;
                                return (
                                    <View key={`image-${index}-${imgIndex}`} style={reportStyles.imageItem}>
                                        <Image src={src} style={reportStyles.image} />
                                        <Text style={reportStyles.caption}>
                                            Figure {imgIndex + 1}: Image from {section.title}
                                            {typeof img !== 'string' && ` | Topic: ${img.topic} | Palette: ${img.palette}`}
                                        </Text>
                                    </View>
                                );
                            })}
                        </View>
                    </View>
                ))}

                <View style={reportStyles.footer} fixed>
                    <Text render={({ pageNumber, totalPages }) => (
                        `Generated on ${new Date().toLocaleDateString()} | Page ${pageNumber} of ${totalPages}`
                    )} />
                </View>
            </Page>
        </Document>
    );
};

export default ReportGenerator;