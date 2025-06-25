#!/usr/bin/env python3
import tf2_geometry_msgs
import rclpy
from geometry_msgs.msg import PointStamped

def point_to_workspace_plane(p, node_time, tf_buffer, from_frame, to_frame):
    """
    Transform point from frame to another 

    Args:
        p (List):  Point with x,y,z - coordinates
        node_time: the node current time 
        tf_buffer (tf2_ros.Buffer): transformation buffer
        from_frame (str): the frame of the point 
        to_frame (str): the frame where the point will be transformed 

    Returns:
        PointStamped: point in the target frame 
    """
    point = PointStamped() 

    point.header.frame_id = from_frame
    point.header.stamp = node_time  

    point.point.x = p[0]
    point.point.y = p[1]
    point.point.z = p[2]
    if to_frame != from_frame: 
        transform = tf_buffer.lookup_transform(to_frame,        
                                            from_frame,      
                                            rclpy.time.Time())

        point = tf2_geometry_msgs.do_transform_point(point, transform)

    return point
