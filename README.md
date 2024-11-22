# SLAM-Person-Mapping

This project uses Simultaneous Localization and Mapping (SLAM) to track and map people in real-time within a changing environment. By combining a stream of footage and machine learning, the system will identify and follow people as they move, updating the map continuously. This project can be applied to areas like indoor navigation, autonomous robots, and smart surveillance, with a focus on reliable human detectio

## Contributors 
- Saad Rafiq
- Mason Melead
- Rose Ochoa
- Colton Richard
- Jimmy Abouhamzeh

## Repo Map
- `src/` - source code
- - `src/person_and_depth.py` - Person and depth detection driver
- - `src/detection_api.py` - Flask API for detections
- - `src/model` - `.caffeemodel` and `.proto.txt` for MobileNet-SSD
- - `src/ros_ws/src/` - git sub repos for LiDAR and Hector-SLAM and source code for project
- - `src/ros_ws/person_mapping/src/slam_handler.py` - SLAM and Pose classes (Python 2)
- - `src/ros_ws/person_mappping/src/main.py` - Driver and GUI code (Python 2)