import React, { createContext, useContext, useState } from "react";
import ROSLIB from "roslib";

interface RosContextType {
    ros: ROSLIB.Ros;
}

const RosContext = createContext<RosContextType | undefined>(undefined);

export const RosProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [ros] = useState(new ROSLIB.Ros({ url: "ws://localhost:9090" }));

    return <RosContext.Provider value={{ ros }}>{children}</RosContext.Provider>;
};

export const useRos = (): RosContextType => {
    const context = useContext(RosContext);
    if (!context) {
        throw new Error("useRos must be used within a RosProvider");
    }
    return context;
};
