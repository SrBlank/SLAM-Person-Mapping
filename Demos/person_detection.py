import cv2
import numpy as np
import pyrealsense2 as rs

# Initialize RealSense pipeline for depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Enable color and depth streams
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
pipeline.start(config)

# Load MobileNet-SSD model (pre-trained weights from Caffe)
net = cv2.dnn.readNetFromCaffe('deploy.prototxt', 'mobilenet_iter_73000.caffemodel')

# List of classes in MobileNet-SSD model
classNames = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", 
    "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", 
    "train", "tvmonitor"
]

# Prepare the capture window
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)

while True:
    # Wait for a new frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    if not color_frame or not depth_frame:
        continue

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # Create blob from image for MobileNet-SSD
    blob = cv2.dnn.blobFromImage(color_image, 0.007843, (300, 300), (127.5, 127.5, 127.5), False, crop=False)
    net.setInput(blob)
    detections = net.forward()

    # Loop through all detections
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.2:  # Filter based on confidence threshold
            # Get the class ID and bounding box
            class_id = int(detections[0, 0, i, 1])
            x1 = int(detections[0, 0, i, 3] * color_image.shape[1])
            y1 = int(detections[0, 0, i, 4] * color_image.shape[0])
            x2 = int(detections[0, 0, i, 5] * color_image.shape[1])
            y2 = int(detections[0, 0, i, 6] * color_image.shape[0])

            # Draw bounding box and label
            if classNames[class_id] == 'person':  # Detect persons only
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{classNames[class_id]}: {confidence:.2f}"
                cv2.putText(color_image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Get depth information for the center of the bounding box
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                depth = depth_frame.get_distance(center_x, center_y)  # Depth in meters
                depth_text = f"Depth: {depth:.2f}m"
                cv2.putText(color_image, depth_text, (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    # Display the results
    cv2.imshow('RealSense', color_image)

    # Exit if ESC is pressed
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Stop the pipeline
pipeline.stop()
cv2.destroyAllWindows()
