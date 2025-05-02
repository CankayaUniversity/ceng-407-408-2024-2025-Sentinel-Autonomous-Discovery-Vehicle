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
        lineHeight: 1.5
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
    }
});