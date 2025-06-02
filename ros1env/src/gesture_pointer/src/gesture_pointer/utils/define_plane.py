#!/usr/bin/env python3
"""
Helper submodule for fetching plane corner coordinates and a mask from
a depth layer using simple GUI 
"""

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX
PROMPTS = ['Click corners in order:', 
           'lower left', 'upper left', 'upper right', 'lower right',
           'Click Q to continue']
PROMPTS_HEIGHT = [40, 64, 88, 112, 136, 160]

# Default order
LEFT_LOW = 0
LEFT_UP = 1
RIGHT_UP = 2
RIGHT_LOW = 3

def collect_corners(event, x, y, depth_layer, recorded_coordinates, 
                    offset_coordinates, offset, width, height):
    """
    Mouse-Click event for collecting the corner image coordinates and 
    corresponding offset coordinates for depth mask. 

    Args:
        event (int): Mouse-Click event on depth frame  
        x (int): image coordinate x for the click 
        y (int): image coordinate y for the click 
        depth_layer (np.ndarray): depth layer as numpy array 
        recorded_coordinates (List): Coordinates for the corners 
        offset_coordinates (List): Offset coordinates for defining a mask  
        offset (int): number of offset pixels
        width (int): stream width 
        height (int): stream height 
    """

    # how many corners already collected 
    recorded_count = len(recorded_coordinates)

    if event == cv2.EVENT_LBUTTONDOWN and recorded_count < 4:

        # Mark the clicked point on the image
        cv2.circle(depth_layer, (x, y), 5, (255, 0, 0))
        cv2.putText(depth_layer, '(' + str(x) + ', ' + str(y) + ')', (x, y), 
                    FONT, 0.75, (255, 0, 0), 2)

        if recorded_count != 0:
            # Draw a line between the previous and current point
            cv2.line(depth_layer, recorded_coordinates[-1], (x, y), 
                     (255, 0, 0), 2)

        # Update the prompt and offset coordinates 
        if recorded_count == LEFT_LOW:
            offset_x = x - offset
            offset_y = y + offset
            cv2.putText(depth_layer, PROMPTS[2], (40, PROMPTS_HEIGHT[2]), FONT, 
                        0.5, (255, 0, 0), 1)

        elif recorded_count == LEFT_UP:
            offset_x = x - offset
            offset_y = y - offset
            cv2.putText(depth_layer, PROMPTS[3], (40, PROMPTS_HEIGHT[3]), FONT, 
                        0.5, (255, 0, 0), 1)

        elif recorded_count == RIGHT_UP:
            offset_x = x + offset
            offset_y = y - offset
            cv2.putText(depth_layer, PROMPTS[4], (40, PROMPTS_HEIGHT[4]), FONT, 
                        0.5, (255, 0, 0), 1)

        elif recorded_count == RIGHT_LOW:
            offset_x = x + offset
            offset_y = y + offset
            # Finalize the polygon
            cv2.line(depth_layer, (x, y), recorded_coordinates[0], 
                     (255, 0, 0), 2)
            cv2.putText(depth_layer, PROMPTS[-1], (40, PROMPTS_HEIGHT[-1]), 
                        FONT, 0.5, (255, 0, 0), 2)

        # ensure the offsets are within the image limits
        offset_x = max(0, min(offset_x, width - 1))
        offset_y = max(0, min(offset_y, height - 1))

        offset_coordinates.append((offset_x, offset_y))
        recorded_coordinates.append((x, y))
        cv2.imshow('image', depth_layer)


def preprocess_depth(depth_layer): 
    """
    Pre-process the depth layer for better GUI experience 
    Args:
        depth_layer (np.ndarray): depth layer as numpy array 

    Returns:
        np.ndarray: pre-processed depth layer 
    """
    # remove the empty values
    depth_layer = cv2.medianBlur(depth_layer,5) 

    # adjust the contrast and brightness  
    depth_8bit = cv2.convertScaleAbs(depth_layer, alpha=0.03) 
    depth_layer = cv2.convertScaleAbs(depth_8bit, alpha=1.5, beta=50)
    return depth_layer


def add_instruction_box(depth_layer,
                        top_left_coordinate=(30,20), 
                        bottom_right_coordinate=(240,180),
                        color=(0,0,0,0.75)): 
    """
    Add an instruction box over the visualized depth layer.

    Args:
        depth_layer (np.ndarray): depth layer as numpy array 
        top_left_coordinate (tuple, optional): Top left coordinate for the 
                                               rectangular instruction box. 
                                               Defaults to (30,20).
        bottom_right_coordinate (tuple, optional): Bottom right coordinate for 
                                                   the rectangular instruction
                                                   box. Defaults to (240,180).
        color (tuple, optional): Color of the instruction box. 
                                 Defaults to (0,0,0,0.75).
    """

    overlay = depth_layer.copy()
    cv2.rectangle(overlay, top_left_coordinate, bottom_right_coordinate, 
                  color[:3], -1)

    # Add the rectangle to the image with transparency
    cv2.addWeighted(overlay, color[3], depth_layer, 1-color[3], 0, depth_layer)
    # Initial instruction text 
    cv2.putText(depth_layer, PROMPTS[0], (40, PROMPTS_HEIGHT[0]), FONT, 
                0.50, (255, 0, 0))

# TODO: Width and height should be specified somewhere / common config 
def get_mask_and_corners_visually(depth_layer, width=1280, height=720, 
                                  offset=20):
    """
    Use image to visually define the workplane coordinates 

    Args:
        depth_layer (np.ndarray): Depth layer as numpy array 
        width (int, optional):  Width of RGB stream. Defaults to 1280.
        height (int, optional): Heigth of RGB stream. Defaults to 720.
        offset (int, optional): Number of offset coordinates for depth mask. 
                                Defaults to 20.
    Returns:
        List, np.ndarray: 2D coordinates for workplane corners, 2D mask for the 
                          workplane area  
    """
    plane_corner_coordinates = []
    offset_coordinates = []

    depth_layer = preprocess_depth(depth_layer)
    add_instruction_box(depth_layer)

    # First corner instruction
    cv2.putText(depth_layer, PROMPTS[1], (40, PROMPTS_HEIGHT[1]), FONT, 0.50, 
                (255, 0, 0))
    cv2.imshow('image', depth_layer)

    # Set callback for collecting the coordinates
    cv2.setMouseCallback('image', lambda event, x, y, flags, 
                         params: collect_corners(event, x, y, depth_layer,
                                                 plane_corner_coordinates,
                                                 offset_coordinates, 
                                                 offset=offset, width=width, 
                                                 height=height))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # All corners are collected 
    if len(plane_corner_coordinates) == 4:
        # Create a mask for the workplace area
        mask = cv2.fillPoly(np.zeros((height, width), dtype=np.uint8), 
                            pts=[np.array(offset_coordinates)],
                            color=(255, 255, 255))

        return plane_corner_coordinates, mask