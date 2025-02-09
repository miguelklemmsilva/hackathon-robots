import cv2
import numpy as np
import onnxruntime as ort

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

        # Load YOLO model
        self.model_path = "../../runs/detect/train18/weights/best.onnx"
        self.session = ort.InferenceSession(self.model_path)

    def get_image(self):
        return self.camera.getImage()

    def get_width(self):
        return self.camera.getWidth()

    def get_height(self):
        return self.camera.getHeight()

    def detect_fire(self):
        """Detect fire using YOLOv8."""
        width = self.get_width()
        height = self.get_height()
        image = self.get_image()

        if image is None:
            return None, False  # Return None for frame, False for fire detection

        # Convert Webots image to OpenCV format
        img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # ✅ Resize and format for YOLO
        yolo_input = cv2.resize(img, (416, 416))  # Resize to (416, 416)
        yolo_input = yolo_input.astype(np.float32) / 255.0
        yolo_input = np.transpose(yolo_input, (2, 0, 1))  # Convert to (3, 416, 416)
        yolo_input = np.expand_dims(yolo_input, axis=0)  # Add batch dimension -> (1, 3, 416, 416)

        # Run YOLO inference
        outputs = self.session.run(None, {"images": yolo_input})
        detections = outputs[0]  # YOLO detections

        fire_detected = False

        for det in detections.T:  # Iterate over detections
            x1, y1, x2, y2 = det[:4]  # Extract bounding box
            conf = det[4]  # Confidence score
            class_scores = det[5:]  # Class scores (probabilities)

            class_id = np.argmax(class_scores)  # Get the class with the highest probability
            class_conf = class_scores[class_id]  # Confidence of that class

            if class_id == 0 and class_conf > 0.9:  # Fire class detected
                fire_detected = True
                x1, y1 = float(x1), float(y1)  # Convert NumPy arrays to scalars

                if self.logger:
                    self.logger.info(f"🔥 Fire detected by {self.device_name} at ({x1:.2f}, {y1:.2f})!")

                # Draw bounding box
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.putText(img, "FIRE!", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                break  # Stop after detecting one fire instance

        return img, fire_detected