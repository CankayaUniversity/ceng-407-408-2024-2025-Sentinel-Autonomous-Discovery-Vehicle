import { useEffect, useState, useRef } from 'react';
import ROSLIB from "roslib";
import { v4 as uuidv4 } from 'uuid';
import { useRos } from '../utils/RosContext';
import { useDispatch, useSelector } from 'react-redux';
import { addNotification } from '../store/reducers/applicationReducer';
import { RootState } from '../store/mainStore';

interface FetchObjectDataProps {
    onObjectDataReceived?: (objectLinks: any[]) => void;
}

const FetchObjectData: React.FC<FetchObjectDataProps> = ({
    onObjectDataReceived,
}) => {
    const [isInitialized, setIsInitialized] = useState<boolean>(false);
    const { ros } = useRos();
    const dispatch = useDispatch();
    const hasFetchedRef = useRef<boolean>(false);

    const generateReport = useSelector((state: RootState) => state.app.generateReport);

    const fetchObjectWithId = useSelector((state: RootState) => state.app.fetchObjectWithId);

    const [responseTopic, setResponseTopic] = useState<ROSLIB.Topic | null>(null);

    const initialize = () => {
        if (!ros || isInitialized) return;

        try {
            const newResponseTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/get_object_links',
                messageType: 'std_msgs/String'
            });

            newResponseTopic.subscribe((responseTopicMessage: ROSLIB.Message) => {
                try {
                    const objectData = JSON.parse((responseTopicMessage as any).data);

                    console.info(objectData);

                    dispatch(addNotification({
                        id: uuidv4(),
                        data: `Object Links Received`,
                        timestamp: new Date().toISOString(),
                        type: "INFO",
                    }));

                    if (onObjectDataReceived) {
                        onObjectDataReceived(objectData);
                    }
                } catch (error) {
                    dispatch(addNotification({
                        id: uuidv4(),
                        data: `Failed to parse object links JSON: ${error}`,
                        timestamp: new Date().toISOString(),
                        type: "ERROR",
                    }));
                }
            });

            setResponseTopic(newResponseTopic);
            setIsInitialized(true);
        } catch (error) {
            dispatch(addNotification({
                id: uuidv4(),
                data: `Failed to initialize object data connection: ${error}`,
                timestamp: new Date().toISOString(),
                type: "ERROR",
            }));
        }
    };

    const fetchObjectData = (id: string = "all") => {
        if (!ros || !isInitialized) {
            dispatch(addNotification({
                id: uuidv4(),
                data: "ROS connection not initialized. Cannot fetch object data.",
                timestamp: new Date().toISOString(),
                type: "WARNING",
            }));
            return;
        }

        const detectedObjectsTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/get_object_request',
            messageType: 'std_msgs/String'
        });

        const message = new ROSLIB.Message({
            data: id
        });

        detectedObjectsTopic.publish(message);

        dispatch(addNotification({
            id: uuidv4(),
            data: `Request sent for object data: ${id}`,
            timestamp: new Date().toISOString(),
            type: "INFO",
        }));

        hasFetchedRef.current = true;
    };

    useEffect(() => {
        if (ros && !isInitialized) {
            initialize();
        }
    }, [ros]);

    useEffect(() => {
        return () => {
            if (responseTopic) {
                responseTopic.unsubscribe();
            }
        };
    }, [responseTopic]);

    useEffect(() => {
        if (generateReport && isInitialized && !hasFetchedRef.current) {
            fetchObjectData("all");
        }
    }, [generateReport, isInitialized]);

    useEffect(() => {
        if (fetchObjectWithId.fetchObject === true) {
            fetchObjectData(fetchObjectWithId.id);
        }
    }, [fetchObjectWithId])

    return <></>;
};

export default FetchObjectData;