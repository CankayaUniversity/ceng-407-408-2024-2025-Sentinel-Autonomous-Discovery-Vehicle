import torch
from ultralytics import YOLO
from rclpy.impl import rcutils_logger


class YoloModel:
    def __init__(self, model="yolov8x.pt"):
        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model).to(device)
        self.logger = rcutils_logger.RcutilsLogger(name="yolo_model")
        self.logger.info(f"Using device: {device}")

    def __call__(self, *args, **kwds):
        return self.model(*args, **kwds)
