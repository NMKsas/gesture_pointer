#!/usr/bin/env python3
# Buffer class for handling point data 
from collections import deque
import numpy as np 

class PointBuffer:
    """
    Buffer class for 3D points
    """
    def __init__(self, buffer_size : int, none_count_limit : int=10):
        """
        Args:
            buffer_size (int): the size of the coordinate buffer 
            none_count_limit (int, optional): Number of consecutive None values
                                              before the buffer average becomes 
                                              invalid. Defaults to 10.
        """
        self.point_buffer = deque(maxlen=buffer_size)
        self.sum = np.zeros(3, dtype=float)
        self.none_count = 0
        self.none_count_limit = none_count_limit

    def _is_full(self) -> bool: 
        """
        True when the buffer is full
        """
        return len(self.point_buffer) == self.point_buffer.maxlen

    def add_point(self, point : list[float]) -> None:
        """
        Add the new intersection coordinate. Count the received None 
        values; when buffer is full, set flag. 

        Args:
            coordinate (List[]): x,y,z coordinate of the intersection
        """
        if point is None: 
            self.none_count +=1
            return 
        
        self.point_buffer.append(point)
        self.none_count = 0

    def get_average(self) -> tuple[float, float, float] | None:
        """
        Return the buffer average as a (x,y,z) tuple 

        Returns:
            tuple | None: a mean point, none if too many non counts encountered
        """
        if not self._is_full():
            return None 
        
        if self.none_count > self.none_count_limit: 
            return None 
        
        array = np.asarray(self.point_buffer)
        return tuple(np.round(array.mean(axis=0),5))
