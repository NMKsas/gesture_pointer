# gesture_pointer tool

This repository contains ROS1 package for `gesture_pointer` tool. The tool uses RGB-D stream and [OpenPose](https://arxiv.org/abs/1812.08008)-based pose estimation node developed in [OpenDR project](https://github.com/opendr-eu/opendr) to localize and publish pointed targets as ROS topics. Full documentation available at [GesturePointer docs](https://nmksas.github.io/gesture_pointer_docs/)
 
https://github.com/user-attachments/assets/82af5a49-0747-4c7f-a834-037e4959ec9f

The tool was developed using ROS1 Noetic distribution; ROS2 Humble draft implementation exists but is yet to be released. The original work was developed for Intel RealSense D415 camera.

Author: Noora Sassali, [`@NMKsas`](https://github.com/NMKsas) 

**Note: This is a pre-release of the repository. Module `snap_to_target` will be included in the near future.**
