#!/usr/bin/env python3
from snap_strategy_lib.snap_action_node import SnapStrategy
from snap_strategy_lib.pointer_sub import PointerSubscriber
import rospy 
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseWithCovariance 
from std_msgs.msg import Int16


TABLE_LEVEL = 0.118     # constant for z-axis, wrt. panda_link0

class GestureLocationStrategy(SnapStrategy): 
    """
    Gesture based strategy. Follow the published pointer values, snap if the 
    pointing gesture remains still enough. 

    Args:
        SnapStrategy (ABC): Strategy class defining the common interface
    """

    def __init__(self, pointer_sub : PointerSubscriber): 
        super().__init__(target_type="location") 

        # TODO: should be in the message! 
        self._default_tf_frame = "panda_link0"
        self._pointer_sub = pointer_sub
        self._threshold = 0.05**2
    
    def reset(self): 
        """
        After successful snapping, reset the buffers 
        """
        self._pointer_sub.clear_buffers()

    def snap_to_target(self, goal): 
        """
        Snap to target using pointing gestures. 

        Returns:
            target_id: int, target id as defined in the grasp detected dict 
            target_pose: PoseWithCovariance, the pose for grasping the object
        """
        # attempt to snap if the buffer is full 
        if (self._pointer_sub.is_buffer_full()): 
            
            max_distance, x_avg, y_avg = self._pointer_sub \
                                        .get_max_squared_range_from_average()


            # snapping is done only if the coordinates remain static enough 
            if max_distance <= self._threshold: 

                # get currently detected objects
                target_id = 200              
                target_pose = PoseWithCovariance() 
                target_pose.pose.position.x = x_avg
                target_pose.pose.position.y = y_avg
                target_pose.pose.position.z = TABLE_LEVEL
                target_pose.pose.orientation.x = 1.0
                target_pose.pose.orientation.y = 0.0
                target_pose.pose.orientation.z = 0.0
                target_pose.pose.orientation.w = 0.0
                
                return target_id, target_pose, False 

        return None, None, False


class SpeechLocationStrategy(SnapStrategy): 
    
    """
    Speech based strategy. Listens to verified locations published by speech 
    transcription.

    Args:
        SnapStrategy (_type_): _description_
    """
    def __init__(self, location_dict): 
        super().__init__(target_type="location") 
        self._speech_verification_sub = rospy.Subscriber("/verified_location", 
                                                         Int16, 
                                                         self.target_callback)
        self._locations_dict = location_dict
        self._verified_location = None 
        self._default_tf_frame = "panda_link0"

    def target_callback(self, msg): 
        self._verified_location = msg.data

    def reset(self): 
        self._verified_location = None 

    def snap_to_target(self, goal): 

        target_id = None
        target_pose = None 

        if self._verified_location is not None: 
            target_id = self._verified_location
            pose = self._locations_dict[str(target_id)]['coordinates']

            target_pose = PoseWithCovariance()
            target_pose.pose.position.x = pose[0]
            target_pose.pose.position.y = pose[1]
            target_pose.pose.position.z = pose[2]
            target_pose.pose.orientation.x = pose[3]
            target_pose.pose.orientation.y = pose[4]
            target_pose.pose.orientation.z = pose[5]
            target_pose.pose.orientation.w = pose[6]

        return target_id, target_pose, False
