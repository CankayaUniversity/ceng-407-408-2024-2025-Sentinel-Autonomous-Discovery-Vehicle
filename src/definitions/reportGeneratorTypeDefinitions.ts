import { StoredMapImage } from "./twoDimensionalMapTypeDefinitions";

export interface DataContentSection {
    title: string;
    content: string;
}

export interface ImageContentSection {
    title: string;
    content: string;
    images: (objectData | StoredMapImage)[];
}

export interface objectData {
    id: string;
    class: string;
    url: string;
}

export interface ReportContent {
    title: string;
    subtitle: string;
    dataSections: DataContentSection[];
    imageSections: ImageContentSection[];
}

export interface ReportGeneratorProps {
    content: ReportContent;
    missionInformation: MissionInformation;
    generatedPaths: GeneratedPath[];
}

export interface MissionInformation {
    type: string,
    objectsToBeDetected: string[],
}

export interface MissionTypeDialogProps {
    open: boolean;
    onClose: () => void;
    onConfirm: (type: string, objects: string[]) => void;
    objectClasses: string[];
}

export interface ReportViewDialogProps {
    open: boolean;
    onClose: () => void;
    missionType: string;
    selectedObjects: string[];
}

export interface GeneratedPath {
    id: string;
    pathUrl: string;
}