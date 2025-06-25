#!/usr/bin/env python3

import numpy as np 
import rclpy
from rclpy.node import Node 
from rclpy.impl import rcutils_logger
from .utils.file_utils import generate_csv
from .constants import CAMERA_TF_FRAME, CORNERS_CSV_FILE, DEFAULT_PATH, MARKERS
from geometry_msgs.msg import PoseStamped
                      
ATTEMPTS = 15
DEBUG_ENABLED = True


class ArucoPoseCollector(Node):

    def __init__(self, marker_id, no_of_samples=20):
        """
        """
        super().__init__('aruco_pose_collector_node')
        self._marker_sub = self.create_subscription(
                        PoseStamped,
                        "/aruco_" + str(marker_id) + "/aruco_single/pose",
                        self.listener_callback,
                        10)
        self._marker_id = marker_id
        self._poses = []
        self._no_of_samples = no_of_samples
        self._mean = None 
        self._variance = None 

    def listener_callback(self, msg):
        self._poses.append([msg.pose.position.x, 
                            msg.pose.position.y,
                            msg.pose.position.z,
                            msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w])
        self.get_logger().info("Added pose sample no. " + str(len(self._poses)))

        if len(self._poses) == self._no_of_samples: 
            self._mean = np.mean(np.array(self._poses), axis=0)
            self._variance = np.var(np.array(self._poses), axis=0)
            self.destroy_subscription(self._marker_sub)
            self.get_logger().info("All samples collected")

    def get_mean(self):
        return self._mean 

    def get_variance(self): 
        return self._variance 

    def debug_print(self): 
        """
        Print the given pose list in ROS1 Pose format. For debugging. 
        Args:
            pose (List[x,y,z,x,y,z,w]): The pose as a List; position (x,y,z) and 
                                        quartenion (x,y,z,w)
        """
        str_print = "\n=====================\n"                   \
                +   "Marker ID: " + str(self._marker_id) + "\n" \
                +   "=====================\n"
        if self._mean is None or self._variance is None: 
            str_print += str(len(self._poses)) + ' samples collected.'
        else: 
            str_print += "Mean pose:\n"                    \
                            +f"\tx:{self._mean[0]}\n"      \
                            +f"\ty:{self._mean[1]}\n"      \
                            +f"\tz:{self._mean[2]}\n"      \
                        +"Mean orientation:\n"             \
                            +f"\tx:{self._mean[3]}\n"      \
                            +f"\ty:{self._mean[4]}\n"      \
                            +f"\tz:{self._mean[5]}\n"      \
                            +f"\tw:{self._mean[6]}\n"      \
                        +"---------------------\n"         \
                        +"Variance pose\n"                 \
                            +f"\tx:{self._variance[0]}\n"  \
                            +f"\ty:{self._variance[1]}\n"  \
                            +f"\tz:{self._variance[2]}\n"  \
                        +"Variance orientation:\n"         \
                            +f"\tx:{self._variance[3]}\n"  \
                            +f"\ty:{self._variance[4]}\n"  \
                            +f"\tz:{self._variance[5]}\n"  \
                            +f"\tw:{self._variance[6]}\n"  \
                        +"=====================" 

        self.get_logger().info(str_print)

def main(args=None):

    rclpy.init(args=args)
    logger = rcutils_logger.RcutilsLogger(name="aruco_logger")
    logger.info("Collecting ArUco pose samples")

    marker_poses = []
    for id in MARKERS: 
        aruco_corner_collector = ArucoPoseCollector(id,5)
        while rclpy.ok(): 
            
            rclpy.spin_once(aruco_corner_collector)
            mean = aruco_corner_collector.get_mean()
            variance = aruco_corner_collector.get_variance()

            if mean is not None and variance is not None: 
                aruco_corner_collector.debug_print()

                # add mean to marker poses list
                data = mean.tolist() 
                data.insert(0, CAMERA_TF_FRAME)
                data.insert(0, str(id))
                marker_poses.append(data)

                aruco_corner_collector.destroy_node()
                break 
    
    logger.info("All corners collected")
    
    generate_csv(CORNERS_CSV_FILE, DEFAULT_PATH,
                ["Aruco ID", "Parent frame ID", 
                "position.x","position.y","position.z",
                "orientation.x", "orientation.y",
                "orientation.z", "orientation.w"],
                marker_poses, False)    
 
    # Destroy the node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()
