import { objectData } from "../../definitions/reportGeneratorTypeDefinitions";
import { StoredMapImage } from "../../definitions/twoDimensionalMapTypeDefinitions";

export const reportTemplateData = {
    content: {
        title: 'Sentinel System Report',
        subtitle: 'Summary of system events',
        dataSections: [
            {
                title: 'Overview',
                content: 'This report includes the generated maps, such as the 2D map and the global costmap. It also includes detected objects categorized by their classes.'
            },
        ],
        imageSections: [
            {
                title: 'Generated Maps',
                content: 'This section presents the generated maps, including the 2D map, global costmap, and other relevant spatial representations produced during the mapping process.',
                images: [] as StoredMapImage[],
            },
            {
                title: 'Summary of Detected Object Types',
                content: 'Summary of detected object types and frequencies will be displayed here.',
                images: [] as objectData[],
            }
        ]
    }
}