#!/usr/bin/env python3
from collections import deque
from geometry_msgs.msg import PointStamped
import numpy as np
import rospy 

class PointerSubscriber():
    """
    Pointer subscriber. The values assumed to be wrt. frame aligned with the 
    plane, values for z coordinate = constant.
    """

    def __init__(self, gesture_topic, buffer_size):
        self._buffer_x = deque(maxlen=buffer_size)
        self._buffer_y = deque(maxlen=buffer_size)
        self._gesture_topic = rospy.Subscriber(gesture_topic, PointStamped, 
                                               self.gesture_callback)

    def gesture_callback(self, msg): 
        """
        Add coordinate values (x,y) to deque buffer 

        Args:
            msg (PointStamped): gesturing value 
        """
        self._buffer_x.append(msg.point.x)
        self._buffer_y.append(msg.point.y)   
    
    def is_buffer_full(self): 
        """
        Check whether both (x,y) buffers are full 

        Returns:
            bool: True if full, False if not.
        """
        return len(self._buffer_x) == self._buffer_x.maxlen and\
        len(self._buffer_y) == self._buffer_y.maxlen 

    def clear_buffers(self): 
        """
        Clear the buffers for new values 
        """
        self._buffer_x.clear()
        self._buffer_y.clear()


    def get_max_squared_range_from_average(self): 
        """
        Calculate the average of the coordinate points, calculate the point 
        distances with respect to average point. Return the maximum distance. 

        Returns:
            float: maximum distance between the points and their average 
        """
        x_coords = np.array(self._buffer_x)
        y_coords = np.array(self._buffer_y)
        x_avg = np.mean(self._buffer_x)
        y_avg = np.mean(self._buffer_y)

        distance_squared_to_mean = (x_coords - x_avg)**2 +(y_coords - y_avg)**2

        return np.max(distance_squared_to_mean), x_avg, y_avg
