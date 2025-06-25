"""
Helper submodule for fetching plane corner coordinates from an image
"""
import numpy as np
import cv2

FONT = cv2.FONT_HERSHEY_SIMPLEX
PROMPTS = ['Click corners in order: left back, right back, right front, left front.',
           'Click Q to continue']
LEFT_BACK = 0
RIGHT_BACK = 1
RIGHT_FRONT = 2
LEFT_FRONT = 3


def collect_corners(event, x, y, img, recorded_coordinates, offset_coordinates, offset, width, height):
    """
    Callback for collecting corner points of the plane.
    Order of collection: Left back, right back, right front, left front
    Args:
        height: Image height in pixels
        width: Image width in pixels
        offset: Offset to define an area around the plane, considered valid for object detection
        offset_coordinates: List [] for previously recorder offset coordinates (x,y)
        event: clicking event
        x: image coordinate x for the click
        y: image coordinate y for the click
        img: the image to be marked
        recorded_coordinates: List [] for previously recorded image coordinates
    """

    recorded_count = len(recorded_coordinates)

    if event == cv2.EVENT_LBUTTONDOWN and recorded_count < 4:

        # Mark the point on the image
        cv2.circle(img, (x, y), 5, (255, 0, 0))
        cv2.putText(img, '(' + str(x) + ', ' + str(y) + ')', (x, y), FONT, 0.5, (255, 0, 0),
                    1)

        if recorded_count != 0:
            # Draw a line between the previous and current point
            cv2.line(img, recorded_coordinates[-1], (x, y), (255, 0, 0), 2)

        offset_x = 0
        offset_y = 0
        if recorded_count == LEFT_BACK:
            offset_x = x - offset
            offset_y = y - offset
        elif recorded_count == RIGHT_BACK:
            offset_x = x + offset
            offset_y = y - offset
        elif recorded_count == RIGHT_FRONT:
            offset_x = x + offset
            offset_y = y + offset
        elif recorded_count == LEFT_FRONT:
            offset_x = x - offset
            offset_y = y + offset
            # Finalize the polygon
            cv2.line(img, (x, y), recorded_coordinates[0], (255, 0, 0), 2)
            cv2.putText(img, PROMPTS[-1], (40, 80), FONT, 0.5, (255, 0, 0), 2)

        # ensure the offsets are within the image limits
        offset_x = max(0, min(offset_x, width - 1))
        offset_y = max(0, min(offset_y, height - 1))

        offset_coordinates.append((offset_x, offset_y))
        recorded_coordinates.append((x, y))
        cv2.imshow('image', img)


def define_plane(image, width=1280, height=720, offset=20, is_depth=True):
    """
    Use image to visually define the plane coordinates
    Args:
        offset: The offset around the plane
        height: The height of the image
        width: The width of the image
        image: CV2 image
        is_depth: Bool, true if the image is depth image
    Returns:
        plane_corner_coordinates: List of plane corners
    """
    plane_corner_coordinates = []
    offset_coordinates = []

    if is_depth:
        # adjust the brightness for depth image
        depth_8bit = cv2.convertScaleAbs(image, alpha=0.03)  # Scale factor
        image = cv2.convertScaleAbs(depth_8bit, alpha=1.5, beta=50)  # Contrast and brightness

    cv2.putText(image, PROMPTS[0], (40, 40), FONT, 1, (255, 0, 0), 2)
    cv2.imshow('image', image)

    # Set callback for collecting the coordinates
    cv2.setMouseCallback('image', lambda event, x, y, flags, params: collect_corners(event, x, y, image,
                                                                                     plane_corner_coordinates,
                                                                                     offset_coordinates, offset=offset,
                                                                                     width=width, height=height))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    if len(plane_corner_coordinates) == 4:
        # Create a mask for evaluating the workplace area
        mask = cv2.fillPoly(np.zeros((height, width), dtype=np.uint8), pts=[np.array(offset_coordinates)],
                            color=(255, 255, 255))

        return plane_corner_coordinates, mask
