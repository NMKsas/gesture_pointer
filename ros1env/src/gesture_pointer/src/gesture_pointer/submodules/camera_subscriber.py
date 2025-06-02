#!/usr/bin/env python3
# Camera subscriber for RGB and Depth streams. Currently supports Intel 
# RealSense D400 cameras, which include aligned_depth_to_color stream. 

from cv_bridge import CvBridge
import message_filters as mf
import pyrealsense2 as rs2
import rospy
import threading 

from sensor_msgs.msg import Image as ROS_Image
from sensor_msgs.msg import CameraInfo


class CameraSubscriber:
    """
    Camera subscriber for depth and rgb streams. Includes methods for 
    projection and deprojection (3D <-> 2D). 
    """

    def __init__(self,
                image_topic="/st_cam/color/image_raw",
                aligned_depth_topic="/st_cam/aligned_depth_to_color/image_raw",
                depth_camera_info_topic="/st_cam/aligned_depth_to_color/camera_info"): 
        
        # initialize topic subscribers 
        self._camera_info_sub = rospy.Subscriber(depth_camera_info_topic, 
                                                 CameraInfo, 
                                                 self.camera_info_cb,
                                                 queue_size=1)
        self._rgb_sub = mf.Subscriber(image_topic, ROS_Image)
        self._depth_sub = mf.Subscriber(aligned_depth_topic, ROS_Image)

        # synchronize depth and rgb streams
        self._ts = mf.ApproximateTimeSynchronizer([self._rgb_sub, 
                                                   self._depth_sub], 
                                                  queue_size=5, slop=20.0, 
                                                  allow_headerless=True)
        self._ts.registerCallback(self.images_cb)

        self._rgb_in = None 
        self._depth_in = None 
        self._camera_intrinsics = None 

        self._bridge = CvBridge() 

        self._lock_rgb = threading.Lock() 
        self._lock_depth = threading.Lock()
        self._lock_intrinsics = threading.Lock() 

        # check if initialization succeeds and topics are received
        rospy.sleep(2)

        if self.get_depth()      is not None and \
           self.get_intrinsics() is not None and \
           self.get_rgb()        is not None:  
            print("Camera subscriber successfully initialized")
        else: 
            raise Exception("Failed to initialize the camera subscriber!" +  
                            "Make sure that the camera node is launched.")            

    def camera_info_cb(self, msg):
        """
        Get camera info from the aligned layer
        """
        if self._depth_in is not None:
            self._camera_intrinsics = rs2.intrinsics()
            self._camera_intrinsics.width = msg.width
            self._camera_intrinsics.height = msg.height
            self._camera_intrinsics.ppx = msg.K[2]
            self._camera_intrinsics.ppy = msg.K[5]
            self._camera_intrinsics.fx = msg.K[0]
            self._camera_intrinsics.fy = msg.K[4]
            if msg.distortion_model == 'plumb_bob':
                self._camera_intrinsics.model = rs2.distortion \
                                                   .modified_brown_conrady
            self._camera_intrinsics.coeffs = [i for i in msg.D]

            # camera information fetched, unregister the subscriber
            self._camera_info_sub.unregister()

    def images_cb(self, image_data, depth_data): 
        """
        Callback for the regular image. The images are saved in cv2 format. 
        """
        with self._lock_rgb: 
            self._rgb_in = self._bridge.imgmsg_to_cv2(image_data, 
                                                      image_data.encoding)
        with self._lock_depth: 
            self._depth_in = self._bridge.imgmsg_to_cv2(depth_data, 
                                                        depth_data.encoding)

    def get_depth(self):
        """
        Return the current depth frame
        """
        with self._lock_depth:
            return self._depth_in
    
    def get_rgb(self): 
        """
        Return the current RGB frame 
        """
        with self._lock_rgb:
            return self._rgb_in

    def get_intrinsics(self):
        """
        Return the camera intrinsics retrieved from depth CameraInfo topic. 
        """

        with self._lock_intrinsics:
            return self._camera_intrinsics 
        
    def project_point_to_pixel(self, point): 
        """
        Project 3D coordinate to 2D image coordinate

        Args:
            point (List[]): 3D coordinate (x,y,z) 

        Returns:
            List[]: 2D coordinate (x,y) in image coordinate system  
        """
        return rs2.rs2_project_point_to_pixel(self.get_intrinsics(), 
                                              list(point))

    def deproject_pixel_to_point(self, pixel, depth_frame=None): 
        """
        Deproject 2D image coordinate to 3D coordinate 

        Args:
            pixel (List[]): 2D image coordinate (x,y)
            depth_frame   : Optional depth frame, if current is not preferred
        Returns:
            List[]: 3D coordinate (x,y,z) in camera coordinate system 
        """
        
        # get current depth frame 
        if depth_frame is None: 
            depth_frame = self.get_depth() 

        # find the depth value of the pixel 
        pixel_depth = depth_frame[pixel[1], pixel[0]]

        # use realsense library to deproject the pixel into 3D point 
        coord = rs2.rs2_deproject_pixel_to_point(self.get_intrinsics(), 
                                                 [pixel[0], pixel[1]],
                                                 pixel_depth)
        return coord