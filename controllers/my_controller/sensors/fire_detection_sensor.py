from controller import Robot, Camera
import onnxruntime as ort
import numpy as np
import cv2

# Load YOLOv8 model
session = ort.InferenceSession("runs/detect/train18/weights/best.onnx")

# Initialize Webots Robot
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get Webots camera
camera = robot.getDevice("camera")
camera.enable(timestep)

# Loop for fire detection
while robot.step(timestep) != -1:
    # Get camera image from Webots
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    # Convert Webots image to OpenCV format
    img_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))
    frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)

    # Preprocess for YOLO
    img = cv2.resize(frame, (640, 640))
    img = img.astype(np.float32) / 255.0
    img = np.expand_dims(img, axis=0)

    # Run YOLO inference
    outputs = session.run(None, {"images": img})

    # Process detections
    fire_detected = False
    detections = outputs[0]  # YOLO detections
    for det in detections:
        x1, y1, x2, y2, conf, cls = det
        if cls == 0 and conf > 0.7:  # Fire detected
            fire_detected = True
            print("🔥 Fire detected in Webots simulation!")
            print("Using extinguisher!!! 🧯🧯🧯")

            # Draw bounding box on the Webots camera image
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(frame, "FIRE!", (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Display the image with detections
    cv2.imshow("Webots Fire Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()