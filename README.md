# gesture_pointer tool

This repository contains `gesture_pointer` modules for ROS1 (Noetic) and ROS2 (Humble, Jazzy), to localize pointed targets on a planar workspace surface. Both ROS1 and ROS2 modules require pose estimation node to detect body keypoints and RGB-D stream to localize and publish pointed targets as ROS topics. 

- ROS1 module utilizes [OpenPose](https://arxiv.org/abs/1812.08008)-based pose estimation node developed in [OpenDR project](https://github.com/opendr-eu/opendr)
- ROS2 module utilizes [YOLO11](https://docs.ultralytics.com/tasks/pose/) pose estimation model by `ultralytics`. 

Full documentation available at [GesturePointer docs](https://nmksas.github.io/gesture_pointer_docs/)
 
https://github.com/user-attachments/assets/82af5a49-0747-4c7f-a834-037e4959ec9f

The original work was developed for Intel RealSense D415 camera. 

Author: Noora Sassali, [`@NMKsas`](https://github.com/NMKsas) 

**Note: This is a pre-release of the repository. Module `snap_to_target` will be included in the near future.**
