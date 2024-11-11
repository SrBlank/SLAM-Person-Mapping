import time
import cv2
import numpy as np
import pyrealsense2 as rs

from typing import List, Dict, Tuple

def get_all_people(
    proto_file: str = './model/deploy.prototxt',
    model_file: str = './model/mobilenet_iter_73000.caffemodel',
    confidence_threshold: float = 0.2
    ) -> List[Dict[str, float]]:
    """Get depth and confidence for all people in RealSense frame.

    Args:
        proto_file: Path to MobileNet-SSD prototxt file.
        model_file: Path to MobileNet-SSD caffemodel file.
        confidence_threshold: Minimum confidence score (0-1).

    Returns:
        List of detected people:
            {
                'depth': float,  # Distance in meters
                'confidence': float,  # Detection confidence (0-1)
            }
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)

    try:
        net = cv2.dnn.readNetFromCaffe(proto_file, model_file)
        frames = pipeline.wait_for_frames(timeout_ms=10000)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return []

        color_image = np.asanyarray(color_frame.get_data())
        blob = cv2.dnn.blobFromImage(color_image, 0.007843, (300, 300), 
                                   (127.5, 127.5, 127.5), False)
        net.setInput(blob)
        detections = net.forward()

        people = []
        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            class_id = int(detections[0, 0, i, 1])
            
            if confidence > confidence_threshold and class_id == 15:
                box = detections[0, 0, i, 3:7]
                h, w = color_image.shape[:2]
                x1, y1, x2, y2 = (box * np.array([w, h, w, h])).astype(int)
                
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                depth = depth_frame.get_distance(center_x, center_y)

                people.append({
                    'depth': depth,
                    'confidence': confidence
                })

        people.sort(key=lambda x: x['depth'])
        return people

    finally:
        pipeline.stop()
"""
def main():
    while True:
        data = get_all_people()
        print(data)
        time.sleep(0.5)

if __name__ == "__main__":
    main()
"""

"""
Expected Output

[{'depth': 0.5260000228881836, 'confidence': 0.9105256199836731}]
[]
[]
[{'depth': 0.27400001883506775, 'confidence': 0.2727588713169098}]
[{'depth': 1.8620001077651978, 'confidence': 0.9919590950012207}]
[{'depth': 1.2780001163482666, 'confidence': 0.9998899698257446}]
[{'depth': 1.255000114440918, 'confidence': 0.942328691482544}]
[{'depth': 1.4800000190734863, 'confidence': 0.9984509944915771}]
[{'depth': 1.4940000772476196, 'confidence': 0.9989467263221741}]
[{'depth': 1.5190000534057617, 'confidence': 0.9986746311187744}]
[]
[{'depth': 1.7810001373291016, 'confidence': 0.9949648976325989}]
[{'depth': 1.7510000467300415, 'confidence': 0.9739450216293335}, {'depth': 2.5830001831054688, 'confidence': 0.92947918176651}]
[{'depth': 1.7610000371932983, 'confidence': 0.9768665432929993}, {'depth': 2.7920000553131104, 'confidence': 0.9095771312713623}]
[{'depth': 1.7660000324249268, 'confidence': 0.9749782085418701}, {'depth': 2.5300002098083496, 'confidence': 0.9720422625541687}]
[{'depth': 1.787000060081482, 'confidence': 0.9496432542800903}, {'depth': 2.616000175476074, 'confidence': 0.9482998251914978}]
[{'depth': 1.7510000467300415, 'confidence': 0.995796799659729}]
"""