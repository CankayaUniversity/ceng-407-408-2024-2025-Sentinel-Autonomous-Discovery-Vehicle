export interface DataContentSection {
    title: string;
    content: string;
}

export interface ImageContentSection {
    title: string;
    content: string;
    images: string[];
}

export interface ReportContent {
    title: string;
    subtitle: string;
    dataSections: DataContentSection[];
    imageSections: ImageContentSection[];
}

export interface ReportGeneratorProps {
    content: ReportContent;
}