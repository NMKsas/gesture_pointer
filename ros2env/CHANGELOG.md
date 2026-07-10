# Changelog 

## [v2.0.0]

### Overview
ROS2 jazzy release for Gesture Pointer tool. 

### New Features
- `planar_pyworkspace` separated as an own package entity
   - workplane definition is fetched from `planar_pyworkspace` using ROS2 service client
   - visualizations are no longer handled by gesturing node (moved to `planar_pyworkspace`)
- overall simplification and clean up for `gesture_pointer` package

### Known Issues
- 

### Installation

To install, follow the [documentation](https://nmksas.github.io/gesture_pointer_docs/) for ROS2 jazzy Release. 

## [v1.0.0-alpha]

### Overview
This is the alpha pre-release of `gesture_pointer` workspace for ROS2. This version includes the initial implementation of the core features. Please note that this is an early version and may contain bugs or incomplete functionality.

### New Features
- packages for `gesture_pointer`, `pose_detection_interfaces` and `pose_keypoint_detector`
- `pose_keypoint_detector` uses YOLO11 model, but has a layer of abstraction to implement keypoint detection with other models  
- Utils for defining corners with ArUco markers

### Known Issues
- Pending `/tf` functionalities and utils 
- The code is not robust for changing the corner order when defining the workplane  

### Installation

To install this pre-release version, follow the [documentation](https://nmksas.github.io/gesture_pointer_docs/). 

---

**Note**: This is a pre-release version and is not recommended for production use. Use at your own risk.

