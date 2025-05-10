import { StyleSheet } from '@react-pdf/renderer';

export const reportStyles = StyleSheet.create({
    noImageText: {
        fontSize: 11,
        fontStyle: 'italic',
        color: '#777',
        textAlign: 'center',
        marginTop: 20,
        marginBottom: 10,
    },
    page: {
        flexDirection: 'column',
        backgroundColor: '#ffffff',
        padding: 30
    },
    header: {
        marginBottom: 20,
        paddingBottom: 10,
        borderBottom: '1px solid #333'
    },
    title: {
        fontSize: 24,
        fontWeight: 'bold',
        marginBottom: 5
    },
    subtitle: {
        fontSize: 16,
        color: '#555',
        marginBottom: 15
    },
    section: {
        margin: 10,
        padding: 10,
    },
    sectionTitle: {
        fontSize: 16,
        fontWeight: 'bold',
        marginBottom: 10
    },
    sectionContent: {
        fontSize: 12,
        lineHeight: 1.5,
        marginBottom: 15
    },
    imageContainer: {
        marginVertical: 15,
        alignItems: 'center'
    },
    image: {
        width: 220,
        height: 150,
        marginBottom: 10
    },
    caption: {
        fontSize: 10,
        color: '#666',
        textAlign: 'center'
    },
    footer: {
        position: 'absolute',
        bottom: 30,
        left: 30,
        right: 30,
        textAlign: 'center',
        color: 'grey',
        fontSize: 12,
        borderTop: '1px solid #ccc',
        paddingTop: 10
    },
    imageGrid: {
        flexDirection: 'row',
        flexWrap: 'wrap',
        justifyContent: 'space-between',
        marginTop: 20
    },
    imageItem: {
        width: '48%',
        marginBottom: 15,
        alignItems: 'center'
    },
    tableContainer: {
        marginTop: 15,
        marginBottom: 15,
        width: '100%',
    },
    tableRow: {
        flexDirection: 'column',
        marginBottom: 20,
        borderBottom: '1px solid #dddddd',
        paddingBottom: 15,
    },
    tablePair: {
        flexDirection: 'row',
        width: '100%',
        marginBottom: 10,
        backgroundColor: '#f9f9f9',
        borderRadius: 4,
        padding: 8,
        borderLeft: '3px solid #3498db',
    },
    tableCell: {
        width: '50%',
        alignItems: 'center',
        padding: 5,
    },
    tableCellDivider: {
        width: 1,
        backgroundColor: '#dddddd',
        alignSelf: 'stretch',
        marginHorizontal: 5,
    },
    tableImage: {
        width: '90%',
        height: 120,
        objectFit: 'contain',
        marginBottom: 8,
        borderRadius: 3,
        border: '1px solid #eeeeee',
    },
    tableRowHeader: {
        flexDirection: 'row',
        backgroundColor: '#f2f2f2',
        paddingVertical: 5,
        paddingHorizontal: 10,
        marginBottom: 5,
        borderRadius: 4,
    },
    tableHeaderText: {
        fontSize: 10,
        fontWeight: 'bold',
        color: '#444',
    },
});