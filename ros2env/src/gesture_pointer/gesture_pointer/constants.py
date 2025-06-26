#!/usr/bin/env python3

# generic constants 
DEFAULT_PATH = "/up/ros2env/src/gesture_pointer/data/"
CACHE_ENABLED = True
CORNER_CACHE_FILE = 'corner_cache'
MASK_CACHE_FILE = 'mask_cache'

# ArUco data collection constants 
CAMERA_TF_FRAME = 'st_cam_color_optical_frame'
CAMERA_RGB_TOPIC = '/camera/st_cam/color/image_raw'
CAMERA_INFO_TOPIC = '/camera/st_cam/color/camera_info'
CORNERS_CSV_FILE = 'corners_mean.csv'
CORNER_MARKER_SIZE = 0.08 # m
MARKERS = [100,101,102,103] 

YOLO11_KEYPOINT_NAMES = ['nose', 
                         'l_eye', 'r_eye', 'l_ear', 'r_ear', 'l_sho', 'r_sho', 
                         'l_elb', 'r_elb', 'l_wri', 'r_wri', 'l_hip', 'r_hip', 
                         'l_knee', 'r_knee', 'l_ank', 'r_ank']

FILTERED_KEYPOINT_DICT = {'l_sho' : 0, 'r_sho' : 1, 
                          'l_elb' : 2, 'r_elb' : 3,
                          'l_wri' : 4, 'r_wri' : 5}