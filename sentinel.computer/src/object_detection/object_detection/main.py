import rclpy
import cv2
import numpy as np
from .perspective_correction import PerspectiveCorrection
from .model import YoloModel


def main(args=None):
    rclpy.init(args=args)

    angle_image = cv2.imread("static/frame1.jpg")
    straight_image = cv2.imread("static/frame.jpg")

    if angle_image is None or straight_image is None:
        return
    angle_image = cv2.convertScaleAbs(angle_image, alpha=1.15, beta=0)

    cv2.imshow("MergedBrightedImage", np.vstack([angle_image, straight_image]))
    cv2.waitKey(0)

    # w=640, h=480
    perspective_correction = PerspectiveCorrection(
        w=angle_image.shape[1], h=angle_image.shape[0], percentage=0.08
    )

    corrected_image = perspective_correction.warp_perspective(angle_image)
    merged_image = perspective_correction.crop_and_merge_images(
        corrected_image, straight_image
    )

    cv2.imshow("CorrectedAndMerged", merged_image)
    cv2.waitKey(0)

    model = YoloModel(model="yolo11x.pt")
    results = model(merged_image)
    annotated_image = results[0].plot()

    cv2.imshow("AnnotatedImage", annotated_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
