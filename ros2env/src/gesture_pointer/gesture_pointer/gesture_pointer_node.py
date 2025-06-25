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

import cv2

import numpy as np
import rclpy
import tf2_ros 

from rclpy.node import Node

from sensor_msgs.msg import Image as ROS_Image
from cv_bridge import CvBridge
from pose_detection_interfaces.msg import Pose2D
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from .submodules.workspace import Workspace
from .submodules.camera_subscriber import CameraSubscriber
from .utils.file_utils import read_corners
from .utils.pose_utils import point_to_workspace_plane
from .utils.visualization_utils import generate_marker_from_point
from .constants import CORNERS_CSV_FILE, DEFAULT_PATH, CORNER_CACHE_FILE, \
                       MASK_CACHE_FILE, FILTERED_KEYPOINT_DICT
from collections import deque

LEFT_SHOULDER = FILTERED_KEYPOINT_DICT['l_sho']
RIGHT_SHOULDER = FILTERED_KEYPOINT_DICT['r_sho']
LEFT_ELBOW = FILTERED_KEYPOINT_DICT['l_elb']
RIGHT_ELBOW = FILTERED_KEYPOINT_DICT['r_elb']
LEFT_WRIST = FILTERED_KEYPOINT_DICT['l_wri']
RIGHT_WRIST = FILTERED_KEYPOINT_DICT['r_wri']

RED_COLOR = [1.0,0.0,0.0,1.0]
GREEN_COLOR = [0.0,1.0,0.0,1.0]


class GesturePointer(Node):

    def __init__(self,
                 camera_tf_frame="st_cam_color_optical_frame",
                 target_tf_frame="st_cam_color_optical_frame",
                 input_poses_topic="/pose_keypoints",
                 output_image_topic="/gesture_projection",
                 from_shoulder_to_wrist=True,
                 cache_enabled=False,
                 predefined_corners=None):
        """
        Args:
            camera_subscriber (CameraSubscriber): Camera subscriber class
            camera_tf_frame (str, optional): Camera /tf frame.
            target_tf_frame (str, optional): /tf frame to publish localized 
                                             points and markers.
            input_poses_topic (str):    Pose estimation topic. 
                                        Defaults to "/opendr/poses".
            output_image_topic (str):   Topic for output projection stream. 
                                        Defaults to "/gesture_projection".
            from_shoulder_to_wrist (bool): Use shoulder-wrist points over 
                                           elbow-wrist points. Defaults to True.
            cache_enabled (bool, optional): When enabled, use previously saved
                                            cache for workspace coordinates. 
                                            Defaults to False.
            predefined_corners (List, optional): List of predefined 4 corner 
                                                 coordinates. Defaults to None.
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
        self._predefined_corners = predefined_corners
        self._conf_threshold = 0.8

        # Define which keypoints are used for projection

        marker_pub_r = self.create_publisher(Marker,
                                             "/gesture_pointer/right_marker", 2)
        pointer_pub_r = self.create_publisher(PointStamped, 
                                              "/gesture_pointer/right_pointer", 
                                              100)
        marker_pub_l = self.create_publisher(Marker,
                                             "/gesture_pointer/left_marker", 2)
        pointer_pub_l = self.create_publisher(PointStamped, 
                                              "/gesture_pointer/left_pointer", 
                                              100)
          
        if from_shoulder_to_wrist:
            self._left_pointer = self.Pointer(LEFT_WRIST,LEFT_SHOULDER,
                                              pointer_pub_l, marker_pub_l,
                                              RED_COLOR)
            self._right_pointer = self.Pointer(RIGHT_WRIST,RIGHT_SHOULDER,
                                              pointer_pub_r, marker_pub_r,
                                              GREEN_COLOR)
        else:
            self._left_pointer = self.Pointer(LEFT_WRIST,LEFT_ELBOW,
                                              pointer_pub_l, marker_pub_l,
                                              RED_COLOR)
            self._right_pointer = self.Pointer(RIGHT_WRIST,RIGHT_ELBOW,
                                              pointer_pub_r, marker_pub_r,
                                              GREEN_COLOR)

        self._cache_enabled = cache_enabled

        self._camera_tf_frame = camera_tf_frame
        self._target_tf_frame  = target_tf_frame

        self._tf_buffer = None 
        self._tf_listener = None
        self._workspace = None 
        self._pose_sub = None 
        self._counter = 0
        if output_image_topic is not None:
            self.image_publisher = self.create_publisher(ROS_Image, 
                                                         output_image_topic, 1)
        else:
            self.image_publisher = None

        self._cv_bridge = CvBridge() 

        # Initialize /tf listener
        self._tf_buffer = tf2_ros.Buffer() 
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        # Subscribe the poses and start processing 
        self._pose_sub = self.create_subscription(Pose2D, 
                                                  self._poses_topic,
                                                  self.initialize, 1)
        
        self.get_logger().info("Gesture pointer started")

    def initialize(self, msg):
        """
        Attempt establishing the workspace 
        """

        if self._camera_sub.is_initialized(): 
            try: 
                self._workspace = Workspace(self._camera_sub,
                                        corner_file_path=DEFAULT_PATH + 
                                                            CORNER_CACHE_FILE,
                                        mask_file_path=DEFAULT_PATH + 
                                                        MASK_CACHE_FILE,
                                        corner_points=self._predefined_corners,
                                        cache_enabled=self._cache_enabled)

                # After successful, get parameters for visualizing projection stream  
                v1, v2 = self._workspace.get_visualizing_norms()
                self._ellipse_delta_x = int(v1 * 0.15)
                self._ellipse_delta_y = int(v2 * 0.15)

                print("Successful workspace initialization")
                self.destroy_subscription(self._pose_sub)
                self._pose_sub = self.create_subscription(Pose2D, 
                                                          self._poses_topic,
                                                          self.callback_poses,
                                                          1) 
                return 
            
            except Exception: 
                print("Something failed")
        
        self._counter += 1
        if self._counter == 100: 
            raise Exception("Failed to initialize the node!")
            

    def callback_poses(self, data):
        """
        Callback to process the input data and publish corresponding topics.
        :param data:    Data containing all the pose key points
        :type data:     opendr_interface.msg.OpenDRPose2D
        """

        # Update and publish pointer based on the pose detection data 
        self.update_pointer(data, self._left_pointer)
        self.update_pointer(data, self._right_pointer)
        self.publish_pointer(self._left_pointer)
        self.publish_pointer(self._right_pointer)

        # publish image feed 
        if self.image_publisher is not None:
            self.publish_visualized_image()

    def update_pointer(self, pose_data, pointer):
        """
        Based on the pose data, update the intersection point to pointer buffer.

        Args:
            pose_data (OpenDRPose2D): OpenPose data for body keypoints 
            pointer (Pointer): pointer entity (left/right)
        """
        # pick the hand gesture key point values
        upper_kp = pose_data.keypoint_list[pointer.get_upper_keypoint_id()]
        lower_kp = pose_data.keypoint_list[pointer.get_lower_keypoint_id()]
        # only do the math if the key points are valid 

        if upper_kp.conf >= self._conf_threshold and \
           lower_kp.conf >= self._conf_threshold:

            pointer.set_keypoints(lower_kp, upper_kp)

            # fetch 3D coordinates
            p0 = self._camera_sub.deproject_pixel_to_point((upper_kp.x, 
                                                            upper_kp.y))
            p1 = self._camera_sub.deproject_pixel_to_point((lower_kp.x, 
                                                            lower_kp.y))

            # calculate intersection point
            plane = self._workspace.get_workspace_plane()
            limits = plane.get_limits() 
            intersection = plane.compute_intersection_point(p0, p1)
            
            if intersection is not None:
                
                # Only pointers outside the workspace None 
                if not (limits[0][0] < intersection[0] < limits[0][1] and \
                        limits[1][0] < intersection[1] < limits[1][1] and \
                        limits[2][0] < intersection[2] < limits[2][1]):
                    intersection = None
                    
            pointer.update_buffer(intersection)


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
            marker = generate_marker_from_point(self._target_tf_frame,
                                                point_on_workspace,
                                                node_time,
                                                2, 0.1,
                                                pointer.get_color())
            
            pointer.get_pointer_publisher().publish(point_on_workspace)
            pointer.get_marker_publisher().publish(marker)


    def publish_visualized_image(self): 
        """
        Publish the image with pointer visualizations 
        """
        # Visualize the results in image
        cv_image = self._camera_sub.get_rgb()

        # Working area defined as a rectangle
        for pair in self._workspace.get_corner_pairs():
            cv2.line(cv_image, (pair[0][0], pair[0][1]),
                    (pair[1][0], pair[1][1]), [0, 0, 255], 2)

        cv_image = self.visualize_pointing_gesture(cv_image, 
                                                   self._left_pointer)
        cv_image = self.visualize_pointing_gesture(cv_image, 
                                                   self._right_pointer)

        # Convert the annotated OpenDR image to ROS2 image message, publish 
        self.image_publisher.publish(self._cv_bridge.cv2_to_imgmsg(cv_image, 
                                                               encoding="rgb8"))

    def visualize_pointing_gesture(self, image, pointer):
        """
        Visualize the vector pointers are based on, e.g., vector from shoulder 
        to wrist keypoint. Draw an ellipse to visualize the area where pointer 
        is directed on the plane.

        Args:
            image (np.array): OpenCV image 
            pointer (Pointer): Pointer entity (left/right)

        Returns:
            image (np.array): image with visualizations 
        """
        [lower_kp, upper_kp] = pointer.get_keypoints()  
        # Draw the pointing gesture pose vector
        if (upper_kp is not None and upper_kp.x != -1 and upper_kp.y != -1 and
            lower_kp is not None and lower_kp.x != -1 and lower_kp.y != 1):
            cv2.arrowedLine(image, (upper_kp.x, upper_kp.y), 
                            (lower_kp.x, lower_kp.y),
                            [0, 255, 0], 2)
        
        intersection_average = pointer.get_pointer() 
        if intersection_average is not None: 
            pointer_2d = self._camera_sub \
                            .project_point_to_pixel(intersection_average)
            # red laser like pointer
            cv2.circle(image, (int(pointer_2d[0]), int(pointer_2d[1])), 2, 
                       [255, 0, 0], 2)

            # scale ellipse upublish_visualized_imagesing plane dimensions
            cv2.ellipse(image, (int(pointer_2d[0]), int(pointer_2d[1])), 
                        (self._ellipse_delta_x, self._ellipse_delta_y),
                        angle=0, startAngle=0, endAngle=360, color=[0, 255, 0], 
                        thickness=1)

        return image

    class Pointer: 
        """
        Pointer class to store the keypoint IDs, relevant publishers and 
        pointer buffer
        """
        def __init__(self, lower_keypoint_id, upper_keypoint_id, pointer_pub,
                     marker_pub, marker_color, buffer_size=5):
            """
            Args:
                lower_keypoint_id (int): ID for the used OpenDRPose2DKeypoint 
                upper_keypoint_id (int): ID for the used OpenDRPose2DKeypoint 
                pointer_pub (Publisher): publisher for the pointer 
                marker_pub (Publisher): publisher for the RViz marker 
                marker_color (List[]): rgba values as list 
                buffer_size (int, optional): size of PointBuffer. Defaults to 5.
            """
            
            self._lower_kp_id = lower_keypoint_id
            self._upper_kp_id = upper_keypoint_id
            self._marker_color = marker_color 
            self._keypoints = [None, None] 

            self._pointer_buffer = self.PointerBuffer(buffer_size)
            self._pointer_pub = pointer_pub
            self._marker_pub = marker_pub

    
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
            self._pointer_buffer.add_pointer(intersection)
                
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

        def get_color(self): 
            """
            Returns the marker color
            """
            return self._marker_color
        
        def set_keypoints(self, lower_kp, upper_kp):
            """
            Set image coordinates for lower and upper keypoints 

            Args:
                lower_kp (Pose2DKeypoint): object {x,y} for 2D coordinates 
                upper_kp (Pose2DKeypoint): object {x,y} for 2D coordinates 
            """
            self._keypoints = [lower_kp, upper_kp]
            
        def get_keypoints(self): 
            """
            Returns list of lower and upper keypoint image coordinates 
            """
            return self._keypoints

        def get_marker_publisher(self): 
            """
            Returns Marker publisher assigned to the pointer 
            """
            return self._marker_pub

        def get_pointer_publisher(self): 
            """
            Returns publisher assigned to the pointer 
            """
            return self._pointer_pub

        class PointerBuffer:
            """
            Buffer class for pointers
            """

            def __init__(self, buffer_size, none_count_limit=10):
                """
            
                Args:
                    buffer_size (int): The size of the coordinate buffers
                    none_count_limit (int, optional): how many None counts are 
                                                      allowed before the pointer 
                                                      becomes invalid 
                """
                self.pointer_buffer_x = deque(maxlen=buffer_size)
                self.pointer_buffer_y = deque(maxlen=buffer_size)
                self.pointer_buffer_z = deque(maxlen=buffer_size)
                self.buffer_size = buffer_size
                self.buffer_full = False
                self.none_count = 0
                self.none_count_limit = none_count_limit
                self.reset_buffer = False

            def add_pointer(self, coordinate):
                """
                Add the new intersection coordinate. Count the received None 
                values; when buffer is full, set flag. 

                Args:
                    coordinate (List[]): x,y,z coordinate of the intersection
                """
                if coordinate is not None:
                    self.pointer_buffer_x.append(coordinate[0])
                    self.pointer_buffer_y.append(coordinate[1])
                    self.pointer_buffer_z.append(coordinate[2])
                    self.none_count = 0
                else:
                    self.none_count += 1

                if len(self.pointer_buffer_x) == self.pointer_buffer_x.maxlen:
                    self.buffer_full = True

            def get_average(self):

                if self.buffer_full and \
                   self.none_count <= self.none_count_limit:
                    
                    return (np.round(np.nanmean(self.pointer_buffer_x),5),
                            np.round(np.nanmean(self.pointer_buffer_y),5),
                            np.round(np.nanmean(self.pointer_buffer_z),5))
                else:
                    return None


def main(args=None):
    rclpy.init(args=args)

    pose_topic = "/pose_keypoints"
    output_image_topic = "/gesture_projection"
    shoulder_to_wrist = True
    corners = read_corners(CORNERS_CSV_FILE, DEFAULT_PATH)

    pointer_node = GesturePointer(
        input_poses_topic=pose_topic,
        output_image_topic=output_image_topic,
        from_shoulder_to_wrist=shoulder_to_wrist, 
        cache_enabled=True,
        predefined_corners=corners
    )
    rclpy.spin(pointer_node)
    # Destroy the node explicitly
    pointer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
