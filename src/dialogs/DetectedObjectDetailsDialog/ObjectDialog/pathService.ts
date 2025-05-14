import ROSLIB from "roslib";

export const generatePath = (
  ros: ROSLIB.Ros,
  odomData: any,
  onSuccess: (imageData: string) => void,
  onError: (error: string) => void
) => {
  const serviceClient = new ROSLIB.Service({
    ros: ros,
    name: "/odom_to_image",
    serviceType: "message_interfaces/srv/OdomToImage",
  });

  const request = new ROSLIB.ServiceRequest({
    odom: odomData,
  });

  // Set timeout for service call
  const timeout = setTimeout(() => {
    onError("Service call timed out. Please try again.");
  }, 12000); // 12 second timeout

  serviceClient.callService(
    request,
    (result) => {
      clearTimeout(timeout);

      if (result.image_data) {
        console.log("Service call successful, received image data");
        onSuccess(result.image_data);
      } else {
        console.error("Service returned empty image data");
        onError("No path could be generated");
      }
    },
    (error) => {
      clearTimeout(timeout);
      console.error("Service call failed:", error);
      onError(`Service error: ${error.toString()}`);
    }
  );
};
