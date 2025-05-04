import torch
from ultralytics import YOLO
from rclpy.impl import rcutils_logger
import cv2
import os
import numpy as np
import urllib.request
from torchvision import models, transforms
import torch.nn.functional as F
from collections import deque
import time
import uuid
import copy



class YoloModel:
    def __init__(self, model="yolov8x.pt"):
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model).to(self.device)
        self.class_names = self.model.names
        self.logger = rcutils_logger.RcutilsLogger(name="yolo_model")
        self.feature_extractor = models.resnet50(pretrained=True)
        self.feature_extractor = torch.nn.Sequential(*list(self.feature_extractor.children())[:-1])
        self.feature_extractor = self.feature_extractor.to(self.device).eval()
        self.SIMILARITY_THRESHOLD = 0.82

        self.known_objects = []
        self.object_count = 0
        self.frame_idx = 0   
        self.last_object_idx = 0
        self.PROCESS_EVERY_N_FRAMES = 2

        self.preprocess = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            ),
        ])
    def __call__(self, *args, **kwds):
        return self.model(*args, **kwds)


    # Use batch processing for feature extraction
    @torch.no_grad()  # More efficient way to disable gradient calculation
    def extract_features(self,image):
        img = self.preprocess(image).unsqueeze(0).to(self.device)
        features = self.feature_extractor(img)
        features = features.view(features.size(0), -1)
        features = F.normalize(features, p=2, dim=1)
        return features.cpu().numpy()

    # Cache similarity computation results
    def is_same_object(self, new_features, known_objects):
        if not known_objects:
            return False
            
        features_array = np.vstack([feat for _, feat,_,_ in known_objects])
        similarities = np.dot(new_features, features_array.T).squeeze()

        return np.any(similarities > self.SIMILARITY_THRESHOLD)


    def process_video_stream(self, frame):
        self.frame_idx += 1

        results = self.model.predict(
            source=frame,
            device=0 if self.device == "cuda" else "cpu",
            verbose=False,
            conf=0.90,
            iou=0.45,
        )

        boxes = results[0].boxes
        frame_unboxed = copy.deepcopy(frame)

        if boxes is None or len(boxes) == 0:
            return frame  # No detections; return original frame

        xyxys = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clss = boxes.cls.cpu().numpy()

        sort_indices = np.argsort(-confs)
        xyxys = xyxys[sort_indices]
        clss = clss[sort_indices]

        h, w, _ = frame.shape
        crops = []
        crop_data = []

        for idx in range(len(xyxys)):
            x1, y1, x2, y2 = map(int, xyxys[idx])

            margin_x = int((x2 - x1) * 0.2)
            margin_y = int((y2 - y1) * 0.2)

            x1 = max(0, x1 - margin_x)
            y1 = max(0, y1 - margin_y)
            x2 = min(w, x2 + margin_x)
            y2 = min(h, y2 + margin_y)

            obj_crop = frame_unboxed[y1:y2, x1:x2]

            if obj_crop.size == 0:
                continue

            crops.append(obj_crop)
            crop_data.append((idx, x1, y1, x2, y2))

            # Draw bounding box on frame
            class_id = int(clss[idx])
            class_name = self.class_names[class_id] if self.class_names else str(class_id)
            color = (0, 255, 0)  # green
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        for i, (obj_crop, (idx, x1, y1, x2, y2)) in enumerate(zip(crops, crop_data)):
            new_feat = self.extract_features(obj_crop)
            class_id = int(clss[idx])
            class_name = self.class_names[class_id] if self.class_names else str(class_id)

            if not self.is_same_object(new_feat, self.known_objects):
                self.known_objects.append((self.object_count, new_feat, obj_crop, class_name))
                self.object_count += 1

        return frame
