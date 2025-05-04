import React from 'react';
import { Page, Text, View, Document, Image } from '@react-pdf/renderer';
import { reportStyles } from './ReportGeneratorStyles';
import { objectData, ReportGeneratorProps } from '../../definitions/reportGeneratorTypeDefinitions';
import { StoredMapImage } from '../../definitions/twoDimensionalMapTypeDefinitions';

const ReportGenerator: React.FC<ReportGeneratorProps> = ({ content, missionInformation }) => {

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

                <View key={`image-section-generated-maps`} style={reportStyles.section}>
                    <Text style={reportStyles.sectionTitle}>{content.imageSections[0].title}</Text>
                    <Text style={reportStyles.sectionContent}>{content.imageSections[0].content}</Text>
                    <View style={reportStyles.imageGrid}>
                        {(content.imageSections[0].images).map((img, imgIndex) => {
                            const src = (img as StoredMapImage).dataUrl;
                            return (
                                <View key={`image-${imgIndex}-${imgIndex}`} style={reportStyles.imageItem}>
                                    <Image src={src} style={reportStyles.image} />
                                    <Text style={reportStyles.caption}>
                                        Figure {imgIndex + 1}: Generated Map | Topic: {(img as StoredMapImage).topic} | Palette: {(img as StoredMapImage).palette}
                                    </Text>
                                </View>
                            );
                        })}
                    </View>
                </View>

                <View key={`image-section-detected-objects`} style={reportStyles.section}>
                    <Text style={reportStyles.sectionTitle}>{content.imageSections[1].title}</Text>
                    <Text style={reportStyles.sectionContent}>{content.imageSections[1].content}</Text>
                    <View style={reportStyles.imageGrid}>
                        {(content.imageSections[1].images as objectData[])
                            .filter((img) => {
                                const cls = img.class.toLowerCase();
                                if (missionInformation.type === "Rescue") {
                                    return ["person", "cat", "dog"].includes(cls);
                                } else if (missionInformation.type === "Find and Detect") {
                                    return missionInformation.objectsToBeDetected
                                        .map((o) => o.toLowerCase())
                                        .includes(cls);
                                }
                                return true;
                            })
                            .map((img, imgIndex) => {
                                const src = img.url;
                                return (
                                    <View key={`image-${imgIndex}`} style={reportStyles.imageItem}>
                                        <Image src={src} style={reportStyles.image} />
                                        <Text style={reportStyles.caption}>
                                            Figure {imgIndex + 1}: Detected Object | {img.class} | {img.id}
                                        </Text>
                                    </View>
                                );
                            })}
                    </View>
                </View>

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