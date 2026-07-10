#!/usr/bin/env python3
from cv_bridge import CvBridge
from std_msgs.msg import Header 
from ultralytics import YOLO 
from rclpy.node import Node 

import rclpy

from pose_detection_interfaces.msg import Pose2D, Pose2DKeypoint
from .pose_keypoint_detector import PoseKeypointDetector

YOLO11_KEYPOINT_DICT = { 0:'nose', 1:'l_eye', 2:'r_eye', 3:'l_ear',
                         4:'r_ear', 5:'l_sho', 6:'r_sho', 7:'l_elb', 8:'r_elb', 
                         9:'l_wri', 10:'r_wri', 11:'l_hip', 12:'r_hip',
                         13:'l_knee', 14:'r_knee', 15:'l_ank', 16:'r_ank'}

DEFAULT_FILTER = { 5 : 'l_sho', 6 : 'r_sho', 7 : 'l_elb',
                   8 : 'r_elb', 9 : 'l_wri', 10: 'r_wri' }

class Yolo11KeypointDetector(PoseKeypointDetector): 
    """
    Class for using YOLO-11 model to detect body keypoints from RGB stream.   

    Args:
        PoseKeypointDetector: abstracted super class for pose keypoint detection
    """
    def __init__(self, node, rgb_topic, rgb_frame,
                 filtered_keypoints=DEFAULT_FILTER):

        super().__init__(node, rgb_topic, rgb_frame)
        self._model = YOLO("yolo11n-pose.pt")
        self._bridge = CvBridge()
        
        self._filtered_keypoints = filtered_keypoints
        self._keypoint_pub = self._node.create_publisher(Pose2D, 
                                                         "pose_keypoints", 1)
        # Initialize Pose message for publisher
        self._msg = Pose2D()
        self._msg.pose_id = 0
        self._msg.header = Header() 
        self._msg.header.frame_id = rgb_frame
        self._node.get_logger().info("YOLO keypoint detector initialized.")
    
    def image_callback(self, msg): 
        """
        Upon new RGB stream msg, detect body keypoints using YOLO-11

        Args:
            msg (ROS_Image): RGB frame 
        """
        cv_image = self._bridge.imgmsg_to_cv2(msg, msg.encoding)

        # make prediction     
        keypoints = self._model.predict(source=cv_image, verbose=False)[0] \
                      .cpu().keypoints
        keypoints_xy = keypoints.xy
        confidences = keypoints.conf

        if keypoints_xy is not None and confidences is not None \
           and len(keypoints) != 0: 
            # include confidence scores  
            keypoints = [[x, y, conf] for ([x, y], conf) in 
                         zip(keypoints_xy[0].tolist(), confidences[0].tolist())]
            self.set_keypoints(keypoints)
            self.publish_poses()

    def publish_poses(self): 
        """
        Publish the filtered keypoints 
        """
        poses = []
        # collect the desired keypoints into keypoint list 
        for key, value in self._filtered_keypoints.items():
            x, y, conf = self.get_keypoint_by_index(key)
            pose = Pose2DKeypoint(
                kpt_name=value,
                conf=float(conf),
                x=int(x),
                y=int(y)
            )
            poses.append(pose)
            
        # update timestamp, publish the message
        self._msg.header.stamp = self._node.get_clock().now().to_msg()
        self._msg.keypoint_list = poses
        self._keypoint_pub.publish(self._msg)
        

def main(args=None):
    rclpy.init(args=args)

    pose_sub = Yolo11KeypointDetector(Node("pose_subscriber"), 
                                      "/camera/st_cam/color/image_raw",
                                      "st_cam_color_optical_frame")
    rclpy.spin(pose_sub._node)

    # Destroy the node explicitly
    pose_sub._node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    