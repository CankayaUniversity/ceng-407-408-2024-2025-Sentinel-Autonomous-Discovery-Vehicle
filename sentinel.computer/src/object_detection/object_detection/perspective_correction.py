import cv2
import numpy as np
from rclpy.impl import rcutils_logger


class PerspectiveCorrection:
    def __init__(self, w: int, h: int, percentage: float):
        self._w = w
        self._h = h

        self._percentage = percentage
        self._src_points = self._set_source_points()
        self._dst_points = self._set_destination_points()
        self._perspective_matrix = cv2.getPerspectiveTransform(
            self._src_points.astype(np.float32), self._dst_points.astype(np.float32)
        )
        self.device = cv2.cuda.getCudaEnabledDeviceCount() > 0

        logger = rcutils_logger.RcutilsLogger(name="perspective_correction")
        logger.info(f"Using device: {"cuda" if self.device == True else "cpu"}")

    def warp_perspective(self, img):
        if self.device == False:
            return cv2.warpPerspective(
                img, self._perspective_matrix, (self._w, self._h)
            )

        # TODO OpenCV with Cuda
        gpu_img = cv2.cuda_GpuMat()
        gpu_img.upload(img)
        gpu_result = cv2.cuda.warpPerspective(
            gpu_img, self._perspective_matrix, (self._w, self._h)
        )
        result = gpu_result.download()
        return result

    def crop_and_merge_images(self, img, img1):
        crop_x = int(self._w * self._percentage)
        cropped_img = img[:, crop_x:]
        cropped_img1 = img1[:, crop_x:]
        return np.vstack([cropped_img, cropped_img1])

    def _set_source_points(self):
        return np.array(
            [
                [0, 0],  # top-left
                [self._w - 1, 0],  # top-right
                [self._w - 1, self._h - 1],  # bottom-right
                [0, self._h - 1],  # bottom-left
            ],
            dtype=np.float32,
        )

    def _set_destination_points(self):
        return np.array(
            [
                [0, 0],  # top-left
                [self._w - 1, 0],  # top-right
                [self._w - 1, self._h - 1],  # bottom-right
                [self._w * self._percentage, self._h],  #  bottom-left
            ],
            dtype=np.float32,
        )
