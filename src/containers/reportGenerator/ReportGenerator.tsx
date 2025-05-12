import React from 'react';
import { Page, Text, View, Document, Image } from '@react-pdf/renderer';
import { reportStyles } from './ReportGeneratorStyles';
import { objectData, ReportGeneratorProps } from '../../definitions/reportGeneratorTypeDefinitions';
import { StoredMapImage } from '../../definitions/twoDimensionalMapTypeDefinitions';

const ReportGenerator: React.FC<ReportGeneratorProps> = ({ content, missionInformation, generatedPaths }) => {
    const pathsPerPage = 2;
    const pathChunks = [];

    if (generatedPaths.length > 0) {
        for (let i = 0; i < generatedPaths.length; i += pathsPerPage) {
            pathChunks.push(generatedPaths.slice(i, i + pathsPerPage));
        }
    }

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
                <View style={reportStyles.footer} fixed>
                    <Text render={({ pageNumber, totalPages }) => (
                        `Generated on ${new Date().toLocaleDateString()} | Page ${pageNumber} of ${totalPages}`
                    )} />
                </View>
            </Page>

            <Page size="A4" style={reportStyles.page}>
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

            {pathChunks.map((pathChunk, chunkIndex) => (
                <Page key={`path-page-${chunkIndex}`} size="A4" style={reportStyles.page}>
                    <View style={reportStyles.header}>
                        <Text style={reportStyles.title}>Path Analysis {pathChunks.length > 1 ? `(${chunkIndex + 1}/${pathChunks.length})` : ''}</Text>
                    </View>

                    <View style={reportStyles.section}>
                        {chunkIndex === 0 && (
                            <>
                                <Text style={reportStyles.sectionTitle}>Original Images vs. Generated Paths</Text>
                                <Text style={reportStyles.sectionContent}>
                                    This section provides a side-by-side comparison of detected objects and their corresponding generated navigation paths.
                                </Text>
                                <Text style={{ ...reportStyles.sectionContent }}>
                                    The navigation paths are calculated based on the odometry data of the Sentinel, where it has detected objects.
                                </Text>
                            </>
                        )}

                        <View style={reportStyles.tableContainer}>
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
                                .filter(img => {
                                    return pathChunk.some(path => path.id === img.id);
                                })
                                .map((img, index) => {
                                    const generatedPath = pathChunk.find(path => path.id === img.id);

                                    return (
                                        <View key={`path-row-${index}`} style={reportStyles.tableRow}>
                                            <View style={reportStyles.tableRowHeader}>
                                                <Text style={reportStyles.tableHeaderText}>
                                                    Object ID: {img.id} | Class: {img.class}
                                                </Text>
                                            </View>
                                            <View style={reportStyles.tablePair}>
                                                <View style={reportStyles.tableCell}>
                                                    <Image
                                                        src={img.url}
                                                        style={reportStyles.tableImage}
                                                    />
                                                    <Text style={reportStyles.caption}>
                                                        Original Image
                                                    </Text>
                                                </View>
                                                <View style={reportStyles.tableCellDivider} />
                                                <View style={reportStyles.tableCell}>
                                                    <Image
                                                        src={generatedPath?.pathUrl || "/api/placeholder/300/200"}
                                                        style={reportStyles.tableImage}
                                                    />
                                                    <Text style={reportStyles.caption}>
                                                        Generated Path
                                                    </Text>
                                                </View>
                                            </View>
                                        </View>
                                    );
                                })}

                            {(content.imageSections[1].images as objectData[])
                                .filter(img => pathChunk.some(path => path.id === img.id))
                                .length === 0 && (
                                    <Text style={reportStyles.noImageText}>
                                        No matching objects with generated paths available.
                                    </Text>
                                )}
                        </View>
                    </View>

                    <View style={reportStyles.footer} fixed>
                        <Text render={({ pageNumber, totalPages }) => (
                            `Generated on ${new Date().toLocaleDateString()} | Page ${pageNumber} of ${totalPages}`
                        )} />
                    </View>
                </Page>
            ))}

            <Page size="A4" style={reportStyles.page}>
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