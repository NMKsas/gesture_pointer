#!/usr/bin/env python3

from abc import ABC, abstractmethod
import threading 
from rclpy.node import Node 
from sensor_msgs.msg import Image as ROS_Image


class PoseKeypointDetector(ABC):
    """
    Abstract class for RGB pose keypoint detection   
    """
    def __init__(self, node : Node, rgb_topic : str, rgb_frame : str):
        self._node = node 
        self._image_sub = self._node.create_subscription(ROS_Image, rgb_topic,
                                                         self.image_callback, 1)
        self._keypoints = None
        self._rgb_frame = rgb_frame
        self._lock = threading.Lock() 

    @abstractmethod
    def image_callback(self, msg):
        """
        RGB stream callback. Logic for detecting poses.

        Args:
            msg (ROS_Image): RGB stream
        """
        pass 

    def set_keypoints(self, keypoints): 
        """
        Setter for keypoint list 

        Args:
            List[]: body keypoints as list 
        """
        if not isinstance(keypoints, list):
            raise TypeError("Keypoints must be in list format. " \
                            "Current format is %s" % type(keypoints))
        with self._lock: 
            self._keypoints = keypoints

    def get_keypoints(self): 
        """
        Getter for keypoint list 

        Returns:
            List[]: body keypoints as list 
        """
        with self._lock: 
            return self._keypoints

    def get_keypoint_by_index(self, index): 
        """
        Return body keypoint by index

        Args:
            index (int): index of the wanted keypoint 

        Returns:
            List[]: image coordinate (x,y) 
        """
        with self._lock: 
            return self._keypoints[index]

