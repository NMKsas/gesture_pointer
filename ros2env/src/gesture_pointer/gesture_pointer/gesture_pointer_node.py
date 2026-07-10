#!/usr/bin/env python
# Gesture pointer node for ROS2 environment.
# Author: Noora Sassali
# Version: 1.0.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import rclpy
import tf2_ros 

from rclpy.node import Node

from cv_bridge import CvBridge
from pose_detection_interfaces.msg import Pose2D
from geometry_msgs.msg import PointStamped
from .submodules.camera_subscriber import CameraSubscriber
from .submodules.point_buffer import PointBuffer
from .utils.pose_utils import point_to_workspace_plane
from planar_pyworkspace_interfaces.srv import GestureVisConfig, GetWorkplane

# keypoint IDs as defined in YOLO11
YOLO_KEYPOINT_DICT = {'l_sho' : 0, 'r_sho' : 1, 
                 'l_elb' : 2, 'r_elb' : 3,
                 'l_wri' : 4, 'r_wri' : 5}

LEFT_SHOULDER = YOLO_KEYPOINT_DICT['l_sho']
RIGHT_SHOULDER = YOLO_KEYPOINT_DICT['r_sho']
LEFT_ELBOW = YOLO_KEYPOINT_DICT['l_elb']
RIGHT_ELBOW = YOLO_KEYPOINT_DICT['r_elb']
LEFT_WRIST = YOLO_KEYPOINT_DICT['l_wri']
RIGHT_WRIST = YOLO_KEYPOINT_DICT['r_wri']


def line_to_plane_intersection(p0 : list[float], p1 : list[float],
                               normal : list[float]) -> np.ndarray | None:
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
            epsilon = 1e-6

            # direction vector of the line
            u = np.array(p1) - np.array(p0)

            # direction vector and the plane normal
            dot = np.dot(normal[:3], u)

            # if the dot product is < epsilon, the line and plane are parallel
            if np.abs(dot) > epsilon:
                
                # solve the factor towards the direction vector u
                t = - (np.dot(normal[:3],p0) + normal[3])\
                       / np.dot(normal[:3], u) 
            
                intersection = p0 + t*u

                return intersection

            return None



class GesturePointer(Node):

    def __init__(self,
                 camera_tf_frame="st_cam_color_optical_frame",
                 target_tf_frame="st_cam_color_optical_frame",
                 input_poses_topic="/pose_keypoints",
                 from_shoulder_to_wrist=True):
        """
        Args:
            camera_tf_frame (str, optional): Camera /tf frame.
            target_tf_frame (str, optional): /tf frame to publish localized 
                                             points and markers.
            input_poses_topic (str):    Pose estimation topic. 
                                        Defaults to "/opendr/poses".
            from_shoulder_to_wrist (bool): Use shoulder-wrist points over 
                                           elbow-wrist points. Defaults to True.
        """
        super().__init__('gesture_pointer_node')

        # Initialize camera subscriber 
        self._camera_sub = CameraSubscriber(self, 
                            "/camera/st_cam/color/image_raw",
                            "/camera/st_cam/aligned_depth_to_color/image_raw", 
                            "/camera/st_cam/aligned_depth_to_color/camera_info")
        self._left_pointer = None
        self._right_pointer = None 
        self._poses_topic = input_poses_topic
        self._conf_threshold = 0.8
        self._workplane_client = self.create_client(GetWorkplane, 'get_workplane')
        self._vis_client = self.create_client(GestureVisConfig, 'enable_gesturing_layer')

        # Define which keypoints are used for projection
        pointer_pub_r = self.create_publisher(PointStamped, 
                                              "/gesture_pointer/right_pointer", 
                                              100)
        pointer_pub_l = self.create_publisher(PointStamped, 
                                              "/gesture_pointer/left_pointer", 
                                              100)
        
        if from_shoulder_to_wrist:
            self._left_pointer = self.Pointer(LEFT_WRIST,LEFT_SHOULDER,
                                              pointer_pub_l)
            self._right_pointer = self.Pointer(RIGHT_WRIST,RIGHT_SHOULDER,
                                              pointer_pub_r)
        else:
            self._left_pointer = self.Pointer(LEFT_WRIST,LEFT_ELBOW,
                                              pointer_pub_l)
            self._right_pointer = self.Pointer(RIGHT_WRIST,RIGHT_ELBOW,
                                              pointer_pub_r)

        self._camera_tf_frame = camera_tf_frame
        self._target_tf_frame  = target_tf_frame

        self._cv_bridge = CvBridge() 

        # Initialize /tf listener
        self._tf_buffer = tf2_ros.Buffer() 
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
    
        self._plane_limits = None 
        self._plane_normal = None 
                
        self._request_workplane()
        self._request_visualization()

        self._pose_sub = self.create_subscription(Pose2D, 
                                                    self._poses_topic,
                                                    self.wait_for_camera_init,
                                                    1) 
        self.get_logger().info("Gesture pointer started")


    def _request_workplane(self) -> None: 
        """
        Calls ROS2 service for workplane normal and limits.

        Raises:
            RuntimeError: 'get_workplane' service is not available
            RuntimeError: 'get_workplane' service call fails 
            RuntimeError: 'get_workplane' service returns no data
        """
        if not self._workplane_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("Required service 'get_workplane' not available")
    
        self.get_logger().info("Send request for workspace")
        workplane_request = GetWorkplane.Request()
        future = self._workplane_client.call_async(workplane_request)
        
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError("Service call 'get_workplane' failed")
        
        response = future.result() 
        if response is None: 
            raise RuntimeError("Failed to obtain workplane initialization data")

        self._plane_limits = [response.limits_x, 
                              response.limits_y, 
                              response.limits_z]
        self._plane_normal = response.plane_normal
        self.get_logger().info("Successful workspace initialization")

    
    def _request_visualization(self)->  None:
        """
        Calls ROS2 service for visualizing gestures. 

        Raises:
            RuntimeError: Service call 'enable_gesturing_layer' fails. 
        """
        
        if not self._vis_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Visualization service 'enable_gesturing_layer' not available")
            return 
        
        # TODO: visualization msg only defines the desired radius for the circle,
        #        the layer gets information of the v./h. normals from the workplane
        vis_request = GestureVisConfig.Request()
        vis_request.radius = 0.10
        future = self._vis_client.call_async(vis_request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if not future.done(): 
            raise RuntimeError("Service call 'enable_gesturing_layer' failed")
        
        response = future.result()
        if response.success: 
            self.get_logger().info("Visualization for gesturing enabled")
        else: 
            self.get_logger().info("Failed to initialize visualization. Disabled.")

    def wait_for_camera_init(self, msg : Pose2D) -> None:
        """
        Poll camera subscriber until initialized, 
        change to pose callback. 

        Args:
            data (Pose2D): Data from Pose keypoint detection  
        """
        if self._camera_sub.is_initialized():
            self.destroy_subscription(self._pose_sub)
            self._pose_sub = self.create_subscription(Pose2D, 
                                                      self._poses_topic,
                                                      self.callback_poses,
                                                      1) 

    def callback_poses(self, data : Pose2D) ->  None:
        """
        Pose detection callback for updating and publishing pointer data

        Args:
            data (Pose2D): Data from Pose keypoint detection  
        """

        # Update and publish pointer based on the pose detection data 
        self.update_pointer(data, self._left_pointer)
        self.update_pointer(data, self._right_pointer)
        self.publish_pointer(self._left_pointer)
        self.publish_pointer(self._right_pointer)

    def update_pointer(self, pose_data : Pose2D, pointer) -> None:
        """
        Localizes the pointing gesture on the given plane 

        Args:
            pose_data (Pose2D): Data from Pose keypoint detection  
            pointer (Pointer): Left/Right Pointer instance 
        """
        # pick the hand gesture key point values
        upper_kp = pose_data.keypoint_list[pointer.get_upper_keypoint_id()]
        lower_kp = pose_data.keypoint_list[pointer.get_lower_keypoint_id()]
        
        # only do the math if the key points are valid 
        if upper_kp.conf >= self._conf_threshold and \
           lower_kp.conf >= self._conf_threshold:
            
            intersection_update = None 
            try: 
                p0 = self._camera_sub.deproject_pixel_to_point((upper_kp.x, 
                                                                upper_kp.y))
                p1 = self._camera_sub.deproject_pixel_to_point((lower_kp.x, 
                                                                lower_kp.y))

                intersection = line_to_plane_intersection(p0, p1, 
                                                          self._plane_normal)

                # pointer target is within the workplane limits 
                if (self._plane_limits[0][0] < intersection[0] < self._plane_limits[0][1] and \
                    self._plane_limits[1][0] < intersection[1] < self._plane_limits[1][1] and \
                    self._plane_limits[2][0] < intersection[2] < self._plane_limits[2][1]):

                    intersection_update = intersection 

            except TypeError as e: 
                # deprojection or intersection produces None 
                self.get_logger().info(e)

            pointer.update_buffer(intersection_update)

    def publish_pointer(self, pointer): 
        """
        Publish the pointer, if the point exists. Requires tf_buffer listener
        to be active.  

        Args:
            pointer (Pointer): Pointer instance to be published 

        """
        intersection = pointer.get_pointer()
        if intersection is not None: 
            x,y,z = [i/1000 for i in intersection]
            marker_pose = [x,y,z]
            
            node_time = self.get_clock().now().to_msg()
            point_on_workspace = point_to_workspace_plane(marker_pose,
                                                          node_time,
                                                          self._tf_buffer,
                                                          self._camera_tf_frame,
                                                          self._target_tf_frame)
            
            pointer.get_pointer_publisher().publish(point_on_workspace)


    class Pointer: 
        """
        Pointer wrapper class to store keypoint IDs, the publisher and 
        a pointer buffer
        """
        def __init__(self, lower_keypoint_id, upper_keypoint_id, 
                     pointer_pub, buffer_size=5):
            """
            Args:
                lower_keypoint_id (int): ID for the used OpenDRPose2DKeypoint 
                upper_keypoint_id (int): ID for the used OpenDRPose2DKeypoint 
                pointer_pub (Publisher): publisher for the pointer 
                buffer_size (int, optional): size of PointBuffer. Defaults to 5.
            """
            
            self._lower_kp_id = lower_keypoint_id
            self._upper_kp_id = upper_keypoint_id

            self._pointer_buffer = PointBuffer(buffer_size)
            self._pointer_pub = pointer_pub
    
        def get_pointer(self): 
            """
            Returns the average intersection value for the pointing gesture 
            """
            return self._pointer_buffer.get_average()

        def update_buffer(self, intersection):
            """
            Update the buffercolor

            Args:
                intersection (List[]): [x,y,z] coordinates of the intersection
            """
            self._pointer_buffer.add_point(intersection)
                
        def get_lower_keypoint_id(self): 
            """
            Returns the used lower keypoint ID 
            """
            return self._lower_kp_id
        
        def get_upper_keypoint_id(self):
            """
            Returns the used upper keypoint ID 
            """ 
            return self._upper_kp_id

        def get_pointer_publisher(self): 
            """
            Returns publisher assigned to the pointer 
            """
            return self._pointer_pub



def main(args=None):
    rclpy.init(args=args)

    #node = Node("gesture_pointer_node")
    pose_topic = "/pose_keypoints"
    shoulder_to_wrist = True

    try: 
        pointer_node = GesturePointer(
            input_poses_topic=pose_topic,
            from_shoulder_to_wrist=shoulder_to_wrist
        )
        rclpy.spin(pointer_node)
        # Destroy the node explicitly
        pointer_node.destroy_node()
    except RuntimeError as e: 
        print(e)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
