# Gesture pointer node for ROS2 
This package contains ROS2 implementation for `gesture_pointer` package. While the original ROS1 implementation uses OpenDR pose estimation node, this implementation 
relies on pretrained `ultralytics YOLO11` model to detect the body keypoints. Full documentation available at [`GesturePointer docs`](https://nmksas.github.io/gesture_pointer_docs/).

### Requirements:

- RealSense D400-series depth camera, pyrealsense2 library
- ultralytics library for YOLO-11 

### Limitations:

- The node is designed for one operator, and does not work correctly if multiple persons are in the camera field of view

- Operator must be properly located in the camera's field of view. The node works only if the OpenPose node is able to detect keypoints with valid depth values 
