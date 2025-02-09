import cv2
import numpy as np
import onnxruntime as ort
import os

class CameraSensor:
    def __init__(self, robot, device_name, time_step, logger=None):
        self.robot = robot
        self.device_name = device_name
        self.time_step = time_step
        self.logger = logger
        self.camera = self.robot.getDevice(self.device_name)

        if self.camera:
            self.camera.enable(2 * self.time_step)
            if self.logger:
                self.logger.info(f"Camera '{self.device_name}' enabled.")
        else:
            if self.logger:
                self.logger.warning(f"Camera '{self.device_name}' not found.")

        # Load YOLO model if available
        self.model_path = "../../runs/detect/train18/weights/best.onnx"
        if os.path.exists(self.model_path):
            self.session = ort.InferenceSession(self.model_path)
            if self.logger:
                self.logger.info(f"YOLO model loaded successfully from {self.model_path}.")
        else:
            self.session = None
            # if self.logger:
            #     self.logger.warning(f"Model not found at {self.model_path}. Skipping ONNX initialization.")

    def get_image(self):
        return self.camera.getImage()

    def get_width(self):
        return self.camera.getWidth()

    def get_height(self):
        return self.camera.getHeight()

    def detect_fire(self):
        """Detect fire using YOLOv8."""
        if self.session is None:
            # if self.logger:
                # self.logger.warning("No YOLO session available. Skipping fire detection.")
            return None, False

        width = self.get_width()
        height = self.get_height()
        image = self.get_image()

        if image is None:
            return None, False  # Return None for frame, False for fire detection

        # Convert Webots image to OpenCV format
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # Resize and format for YOLO input
        yolo_input = cv2.resize(img, (416, 416))
        yolo_input = yolo_input.astype(np.float32) / 255.0
        yolo_input = np.transpose(yolo_input, (2, 0, 1))  # (3, 416, 416)
        yolo_input = np.expand_dims(yolo_input, axis=0)  # (1, 3, 416, 416)

        # Run YOLO inference
        try:
            outputs = self.session.run(None, {"images": yolo_input})
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error during YOLO inference: {e}")
            return img, False

        detections = outputs[0]
        fire_detected = False

        # Iterate over detections
        for det in detections:
            x1, y1, x2, y2 = det[:4]
            class_id = int(det[5])  # Assuming class_id is at index 5
            conf = det[4]  # Confidence score

            if class_id == 0 and conf > 0.9:  # Fire class with high confidence
                fire_detected = True
                if self.logger:
                    self.logger.info(f"🔥 Fire detected at ({x1:.2f}, {y1:.2f}) with confidence {conf:.2f}.")

                # Draw bounding box
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(img, "FIRE!", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                break  # Stop after one detection

        return img, fire_detected