#!/usr/bin/env python3
from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Point


def generate_marker_from_point(frame_id, p, id, scale=0.1, color=None, type=2): 
    """
    Generates an rviz marker for the given point
    Args:
        frame_id (str):          tf frame identifier of the point
        p (PointStamped):        geometry message for Point, includes coordinate
        id (int):                identifier for the marker 
        scale (float, optional):     The scale of the marker. Defaults to 0.1.
        color ([r,g,b,a], optional): RGB color and alpha channel as floats 
                                     [0,1]. If none, green color is used.
        type (int): The type of the marker. Defaults to spherical marker. 

    Returns:
        _type_: _description_
    """
    
    marker = Marker() 
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now() 
    marker.type = type
    marker.id = id 


    # Set the scale of the marker
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale

    # Set the coordinate of the marker
    marker.pose.position.x = p.point.x
    marker.pose.position.y = p.point.y
    marker.pose.position.z = p.point.z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    if color is None: 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
    else: 
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    return marker

def generate_line_strip_marker(frame_id, id, point_a, point_b, 
                               scale=0.1, color=None): 

        marker = Marker()
        marker.header.frame_id = frame_id 
        marker.id = id 
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # marker color
        if color is None: 
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        else: 
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = point_a[0]
        first_line_point.y = point_a[1]
        first_line_point.z = point_a[2]
        marker.points.append(first_line_point)
        
        # second point
        second_line_point = Point()
        second_line_point.x = point_b[0]
        second_line_point.y = point_b[1]
        second_line_point.z = point_b[2]
        marker.points.append(second_line_point)

        # Publish the Marker
        return marker 
