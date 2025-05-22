#!/usr/bin/env python3
from snap_strategy_lib.detector_sub import DetectionSubscriber
import rospy 
WORKSPACE_LIMIT_X = [0.30, 0.75]
WORKSPACE_LIMIT_Y = [-0.35, 0.35]
ACCEPTED_OBJECTS_DICT = {'0': 'allen key', '1': 'screwdriver', '2': 'big bolt',
                         '3': 'pipe', '4': 'medium bolt', '5': 'small bolt', 
                         '6': 'bolt hole', '7': 'cover', '8': 'elbow', 
                         '9': 'elbow target', '10': 'cover target', 
                         '11': 'metallic box'}
ACCEPTED_OBJECTS_DICT = {'2': 'big bolt', '5': 'small bolt'}

class DetectionFilter:
    """ 
    Helper class to filter wanted detections out of other rubbish 
    """

    def __init__(self, detection_subs : DetectionSubscriber,
                 accepted_objects_dict=ACCEPTED_OBJECTS_DICT,
                 workspace_limits=[WORKSPACE_LIMIT_X, WORKSPACE_LIMIT_Y]):
        
        self._detections = detection_subs
        self._filtered_detections = {}
        self._workspace_limits = workspace_limits 
        self._accepted_objects_dict=accepted_objects_dict

    def update_filtered_detections(self): 
        """
        Filter the detections based on workspace limits and 
        accepted categories, update filtered lists when called 
        """
        detections = self._detections.get_detections()

        # clear old values 
        self._filtered_detections.clear()

        for (id, pose_dict) in detections.items():

            # filter needed classes 
            if id in self._accepted_objects_dict: 
                self._filtered_detections[id] = []

                # make sure the poses are fresh and within workspace limits
                for index, (timestamp, pose) in pose_dict.items(): 
                    if self._workspace_limits[0][0] <= pose.position.x and \
                       self._workspace_limits[0][1] >= pose.position.x and \
                       self._workspace_limits[1][0] <= pose.position.y and \
                       self._workspace_limits[1][1] >= pose.position.y and \
                       (rospy.Time.now() - timestamp).to_sec() <= 1: 
                        
                        # update lists 
                        self._filtered_detections[id].append(pose)
            
    def get_target_by_id(self, id): 
        """
        Returns the first item, stored by given ID. 
        """
        if id not in self._filtered_detections: 
            return None 
        return self._filtered_detections[id]

    def get_filtered_detections(self):
        """
        Return filtered detections  
        """ 
        return self._filtered_detections        
