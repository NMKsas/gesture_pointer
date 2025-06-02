#!/usr/bin/env python3
"""
Helper function for establishing 2D depth mask and offset corners of the plane. 
Utilized, when pre-defined workplane coordinates are used. 
"""

import cv2
import numpy as np

# Default order
LEFT_LOW = 0
LEFT_UP = 1
RIGHT_UP = 2
RIGHT_LOW = 3

def get_mask_and_offset_corners(corner_points, width=1280, height=720, offset=20): 
    """
    Generate depth mask and offset corners for the workplane, using 
    pre-defined coordinates.

    Args:
        corner_points (List): 2D image coordinates for corners 
        width: The width of the image
        height: The height of the image
        offset: The offset around the plane
    Returns:
        List, np.array: list of offset corners, depth mask as numpy array 
    """

    for i in range(len(corner_points)): 
        
        if i == LEFT_LOW: 
            offset_x = corner_points[i][0] - offset - 20
            offset_y = corner_points[i][1] + offset + 10
            previous = RIGHT_LOW
        elif i == LEFT_UP:
            offset_x = corner_points[i][0] - offset
            offset_y = corner_points[i][1] - offset
            previous = LEFT_LOW
        elif i == RIGHT_UP:
            offset_x = corner_points[i][0] + offset
            offset_y = corner_points[i][1] - offset
            previous = LEFT_UP
        elif i == RIGHT_LOW:
            offset_x = corner_points[i][0] + offset + 20
            offset_y = corner_points[i][1] + offset + 10
            previous = RIGHT_UP


        out_x = (width-1) - int(offset_x)
        out_y = (height-1) - int(offset_y)

        # One of the points is outside of image frame 
        if out_x < 0 or out_y < 0: 

            # set larger offset value 0 or frame maximum, scale another
            if out_x < out_y: 
                
                # the coordinate axis x has larger offset; set 0 / frame maximum 
                offset_x = max(0, min(int(offset_x), width - 1)) 

                # ensure the proportions remain consistent with the previous point, i.e
                # (x - x_offset) / (x - x_corrected) = (y - y_offset) / (y - y_corrected)
                delta_x = corner_points[i][0] - corner_points[previous][0]
                delta_cx = offset_x - corner_points[previous][0]
                delta_y = corner_points[i][1] - corner_points[previous][1]
                offset_y = (delta_x*corner_points[i][1] - delta_y*delta_cx)/delta_x
            else: 
                # the coordinate axis y has larger offset; set 0 / frame maximum 
                offset_y = max(0, min(int(offset_y), height - 1)) 

                # ensure the proportions remain consistent with the previous point, i.e
                # (x - x_offset) / (x - x_corrected) = (y - y_offset) / (y - y_corrected)
                delta_y = corner_points[i][1] - corner_points[previous][1]
                delta_cy = offset_y - corner_points[previous][1]
                delta_x = corner_points[i][0] - corner_points[previous][0]
                offset_x = (delta_y*corner_points[i][0] - delta_x*delta_cy)/delta_y

        corner_points[i][0] = int(offset_x)
        corner_points[i][1] = int(offset_y)

    mask = cv2.fillPoly(np.zeros((height, width), dtype=np.uint8), 
                        pts=[np.array(corner_points)],
                        color=(255, 255, 255))
    return corner_points, mask
