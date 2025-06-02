#!/usr/bin/env python3
import rospy 
from math import sqrt 
from snap_strategy_lib.detection_filter import DetectionFilter
from snap_strategy_lib.pointer_sub import PointerSubscriber
from snap_strategy_lib.snap_action_node import SnapStrategy
from geometry_msgs.msg import PoseWithCovariance 
from std_msgs.msg import Int16 

DEFAULT = 100 # metres, large enough init value 
MANY_OBJECTS = -1

class GestureObjectStrategy(SnapStrategy):
    """
    Gesture based strategy. Follow the published pointer values, snap if the 
    pointing gesture remains still enough. 

    Args:
        SnapStrategy (ABC): Strategy class defining the common interface
    """

    def __init__(self, detection_filter : DetectionFilter,
                 pointer_sub : PointerSubscriber): 
        super().__init__(target_type="object") 
        self._detection_filter = detection_filter
        self._pointer_sub = pointer_sub
        self._threshold = 0.05**2
    
    def reset(self): 
        """
        After successful snapping, clear the buffers. 
        """
        self._pointer_sub.clear_buffers()


    def snap_to_target(self, goal): 
        """
        Snap to target using pointing gestures. 

        Returns:
            target_id: int, target id as defined in the grasp detected dict 
            target_pose: PoseWithCovariance, the pose for grasping the object
            is_group: Snapping to group. False by default. 
        """

        if (self._pointer_sub.is_buffer_full()): 
            
            max_distance, x_avg, y_avg = self._pointer_sub\
                                             .get_max_squared_range_from_average()

            # snapping is done only if the coordinates remain static enough 
            if max_distance > self._threshold: 
                return None, None
            else: 
                # get currently detected objects
                self._detection_filter.update_filtered_detections() 
                objects = self._detection_filter.get_filtered_detections()

                # initialize snapped target
                target_id = None                 
                target_pose = PoseWithCovariance()
                
                # initialize with large distance
                min_distance_object = DEFAULT
                
                if goal.group_id != -1: 
                    objects = {goal.group_id : objects[goal.group_id]}
                for id in objects: 
                    for i in range(len(objects[id])):
                        # calculate distance to each recognized target 
                        distance = sqrt((objects[id][i].position.x - x_avg)**2
                                       +(objects[id][i].position.y - y_avg)**2)
                        # choose the closest target 
                        if  distance < min_distance_object:  
                            min_distance_object = distance
                            target_id = id
                            target_pose.pose = objects[id][i] 
                return target_id, target_pose, False 

        return None, None, False 
        

class SpeechObjectStrategy(SnapStrategy): 
    """
    Speech based strategy. Listens to verified objects published by speech 
    transcription.
    """
    def __init__(self, detection_filter : DetectionFilter): 
        super().__init__(target_type="object") 
        self._detection_filter = detection_filter
        self._speech_verification_sub = rospy.Subscriber("/verified_object", 
                                                         Int16, 
                                                         self.target_callback)
        self._verified_object = None 
        self._timestamp = None

    def target_callback(self, msg): 
        """
        Callback for verified object message. Saves an approximate time of 
        arrival to filter too old messages in the algorithm. 
        Args:
            msg (std_msgs/String): The name of the verified object
        """
        # Save the time of arrival to filter too old messages
        self._timestamp = rospy.Time.now()
        self._verified_object = msg.data
        
    def reset(self): 
        """
        Reset the verified object and timestamp to None 
        """
        self._verified_object = None 
        self._timestamp = None 

    def snap_to_target(self, goal): 
        """
        When speech recognition publishes a verified object from the set of 
        known targets, see if the object is detected in the workspace and snap
        to target

        Returns:
            int, PoseWithCovariance: Target ID and pose 
        """

        target_id = None               
        is_group = False  
        target_pose = None 
        
        if self._verified_object is not None and self._timestamp is not None:
            current_time = rospy.Time.now() 

            # The message has to be fresh, < 2s old 
            if (current_time - self._timestamp).to_sec() >= 2: 
                self.reset()
            else:  
                rospy.loginfo("[snap_speech] Verified object received by speech")
                self._detection_filter.update_filtered_detections() 
                target_id = self._verified_object
                pose_list = self._detection_filter \
                                       .get_target_by_id(self._verified_object)
                
                if pose_list is not None:
                    target_pose = PoseWithCovariance()
                    
                    # check if multiple detections of same category exist
                    if len(pose_list) == 1:
                        target_pose.pose = pose_list[0] 
                    else: 
                        target_id = target_id 
                        is_group = True 

        return target_id, target_pose, is_group 


