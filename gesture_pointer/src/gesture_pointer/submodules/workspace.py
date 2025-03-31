#!/usr/bin/env python3
# Workspace class for defining a 2D plane for gesturing. 

import numpy as np
import cv2
import copy

LEFT_LOW = 0
LEFT_UP = 1
RIGHT_UP = 2
RIGHT_LOW = 3

from submodules.camera_subscriber import CameraSubscriber
from utils.define_plane import get_mask_and_corners_visually
from utils.aruco_depth_mask import get_mask_and_offset_corners

class Workspace:
    """
    Workspace with a 2D workplane. Defined either by using RGB-D stream or 
    pre-defined 3D coordinates wrt. camera coordinate frame.
    """

    def __init__(self, camera_subscriber : CameraSubscriber, 
                 corner_file_path : str, mask_file_path : str, 
                 corner_points=None, cache_enabled=False):
        """
        Workspace initializer

        Args:
            camera_subscriber (CameraSubscriber): Camera subscriber for RGB-D 
                                                  stream 
            corner_file_path (str): file path for a corner cache
            mask_file_path (str): file path for a mask cache 
            corner_points (List, optional):  List of predefined 4 corner 
                                             coordinates.
            cache_enabled (bool, optional): True if the existing cache should be 
                                            used; false otherwise.
        """
        self._camera_sub = camera_subscriber

        # File paths for cache
        self._cache_enabled = cache_enabled
        self._cf_path = corner_file_path
        self._mf_path = mask_file_path

        # Workspace plane and the depth mask 
        self._workspace_plane = None
        self._depth_mask = None
        self._depth_frame = None 

        # pre-defined points if given 
        self._original_corners = copy.deepcopy(corner_points)

        # Visualizing aids 
        self._h_norm = None # horizontal width   
        self._v_norm = None # vertical heigth
        self._corner_pairs = None

        self._debug_enabled = True 
        self._cache_enabled = cache_enabled 
        self.set_workspace() 

    def define_workspace_corners(self): 

        # get corners and binary mask for the depth frame 
        corners_3d = []
        corners_2d = []
        self._depth_frame = cv2.medianBlur(self._camera_sub.get_depth(),5) 

        if self._original_corners is not None: 
            corners_3d = self._original_corners
            for c in corners_3d: 
                corners_2d.append(self._camera_sub.project_point_to_pixel(c))
                # m to mm  
                c[0]*=1000
                c[1]*=1000
                c[2]*=1000
            corners_2d, self._depth_mask = \
                get_mask_and_offset_corners(corners_2d)
        else: 

            # cache values stored 
            if self._cache_enabled: 
                corners_2d, self._depth_mask = self.load_workspace_cache() 

            if corners_2d is None or self._depth_mask is None:  
                # use visual tool to define the workspace 
                corners_2d, self._depth_mask = \
                    get_mask_and_corners_visually(self._depth_frame)

            for c in corners_2d:
                corners_3d.append(self._camera_sub
                                      .deproject_pixel_to_point(c, 
                                                            self._depth_frame))
            self._original_corners = corners_3d

        if self._debug_enabled:  
            print("-----------------------------------")
            print("2D coordinates of workspace corners")
            print("-----------------------------------")
            for c2d in corners_2d: 
                print(str(c2d))
            print("-----------------------------------")
            print("3D coordinates of workspace corners")
            print("-----------------------------------")
            for c3d in self._original_corners: 
                print(str(c3d))
            print("===================================")

        return corners_2d, corners_3d

    def load_workspace_cache(self):
        """
        Load values for corners and depth mask from cache instead of defining 
        them visually all over again. 

        Returns:
            list, numpy.ndarray: corners as list of tuples (x,y), binary 
                                 mask for the depth frame
        """

        try:
            with open(self._cf_path, 'rb') as corner_file:
                corners = np.load(corner_file)
            with open(self._mf_path, 'rb') as mask_file:
                depth_mask = np.load(mask_file)
            print("Cache loaded.")
            return corners, depth_mask 

        except OSError:
            print("No cache files.")
            return None, None 

    def set_workspace(self):
        """
        Set the current workspace, by defining the plane coordinates. 
        Create necessary vectors for visualization
        """
        
        corners_2d, corners_3d = self.define_workspace_corners()

        # Initialize workspace plane
        self._workspace_plane = self.WorkspacePlane(corners_3d)

        # Add corner pairs for visualization purposes
        self._corner_pairs = [[corners_2d[i], 
                            corners_2d[(i + 1) % len(corners_2d)]]
                            for i in range(len(corners_2d))]

        # define norms for visualization purposes 
        self._h_norm = np.linalg.norm(np.array(corners_2d[RIGHT_UP]) - 
                                      np.array(corners_2d[LEFT_UP]))
        self._v_norm = np.linalg.norm(np.array(corners_2d[RIGHT_LOW]) - 
                                      np.array(corners_2d[RIGHT_UP]))
        
        # save the cache 
        self.save_workspace_cache(corners_2d, self._depth_mask)
        return True 

    def save_workspace_cache(self, corners, mask):
        """
        Saves the workspace coordinate list and depth mask in cache files. 

        Args:
            corners (list): List of tuples, coordinates (x,y) in 2D 
            mask (numpy.ndarray): Binary mask for depth frame which was used to 
                                  define the workspace 
        """

        try:
            with open(self._cf_path, 'wb') as corner_file:
                np.save(corner_file, corners)
            with open(self._mf_path, 'wb') as mask_file:
                np.save(mask_file, mask)
            print("Cache saved successfully.")

        except OSError:
            print("Failed saving the workspace cache")

    def get_original_depth_frame(self):
        """
        Returns the depth frame which was used to set the workspace 

        Returns:
            numpy.ndarray: depth frame 
        """
        return self._depth_frame

    def get_depth_mask(self):
        """
        Binary mask for the depth frames

        Returns:
            numpy.ndarray: Binary mask for filtering the workspace, region of 
                           interest 
        """
        return self._depth_mask

    def get_original_corners3d(self):
        return self._original_corners

    def get_corner_pairs(self):
        """
        List of consecutive corner pairs. Using L for left, R for right, B for 
        back, F for front; e.g. [[LB, RB],[RB, RF], [RF, LF], [LF, LB]].
        Aid for visualizing the border lines of the workplane 

        Returns:
            list: list of lists, [[(x,y), (x,y)]...]
        """
        return self._corner_pairs

    def get_workspace_plane(self):
        """
        Returns inner class instance, WorkspacePlane

        Returns:
            WorkspacePlane: 3D workspace plane 
        """
        return self._workspace_plane

    def get_visualizing_norms(self):
        """
        Returns norms for scaling circular areas on plane.

        Returns:
            float, float: horizontal norm: the Euclidean norm between left back
                          and right back corners, 
                          vertical norm: the Euclidean norm between right back 
                          and right front corners
        """
        return self._h_norm, self._v_norm
    

    class WorkspacePlane:
        epsilon = 1e-6

        def __init__(self, corners):
            """
            3D workspace plane
            Args:
                corners: Corners (x,y,z) of the workspace plane as a List[]
            """

            self.corners = np.array(corners)
            self.workspace_limits = self.define_limits()
            self.normal = self.compute_plane_normal()
            self.epsilon = 1e-8

        def compute_plane_normal(self):
            """
            Calculate the norm of the plane, using three points
            on the plane.
            """

            p1 = self.corners[LEFT_UP]
            p2 = self.corners[RIGHT_UP]  
            p3 = self.corners[RIGHT_LOW] 

            normal_v = np.cross(p1 - p2, p3 - p2)

            # Normalize the normal vector
            normal = normal_v / (np.linalg.norm(normal_v) + self.epsilon)

            # Calculate the constant term for the plane equation
            const = -np.dot(normal, p3)

            # The plane equation in normal form
            return np.append(normal, const)

        def compute_intersection_point(self, p0, p1):
            """
            A function to calculate intersection between a line and the 
            workspace plane
            Args:
                p0: The starting point of the line
                p1: The ending point of the line

            Returns:
                np.array: The intersection point of the line and the plane, 
                if it exists. If the line and plane are parallel, returns None.
            """
            # direction vector of the line
            u = np.array(p1) - np.array(p0)

            # direction vector and the plane normal
            dot = np.dot(self.normal[:3], u)

            # if the dot product is < epsilon, the line and plane are parallel
            if np.abs(dot) > self.epsilon:
                
                # solve the factor towards the direction vector u
                t = - (np.dot(self.normal[:3],p0) + self.normal[3])\
                       / np.dot(self.normal[:3], u) 
            
                intersection = p0 + t*u

                return intersection

            return None

        def define_limits(self):
            """
            Define the workspace limits, based on the corner 3D coordinates
            """
            min_x = np.min(self.corners, axis=0)[0] 
            min_y = np.min(self.corners, axis=0)[1] 
            min_z = np.min(self.corners, axis=0)[2]

            max_x = np.max(self.corners, axis=0)[0] 
            max_y = np.max(self.corners, axis=0)[1] 
            max_z = np.max(self.corners, axis=0)[2]
            return [[min_x, max_x], [min_y, max_y], [min_z, max_z]]

        def get_plane_normal(self):
            """
            Returns: Plane function in a normal form
            """
            return self.normal

        def get_limits(self):
            """
            Returns: Workspace limits in 3D coordinates
            """
            return self.workspace_limits
