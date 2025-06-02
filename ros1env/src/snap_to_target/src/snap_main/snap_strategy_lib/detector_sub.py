#!/usr/bin/env python3
import rospy
import threading
from math import sqrt
from vision_msgs.msg import ObjectHypothesisWithPose

# TODO: Check whether this is actually correct list or not 
OBJECT_DICT = {'0': 'allen key', '1': 'screwdriver', '2': 'big bolt',
               '3': 'pipe', '4': 'medium bolt', '5': 'small bolt', 
               '6': 'bolt hole', '7': 'cover', '8': 'elbow', 
               '9': 'elbow target', '10': 'cover target', '11': 'metallic box'}

class DetectionSubscriber:

    def __init__(self, threshold=0.0):
        rospy.Subscriber('/opendr/grasp_detected', ObjectHypothesisWithPose, 
                         self.detection_cb)
        # confidence score for detection
        self._threshold = threshold

        # Lock to prevent race condition between detection_cb and get_detection
        self._lock = threading.Lock() 
        self._detections = {}
        self._debug_enabled = False 
        self._distance_threshold = 0.03

    def detection_cb(self, data):
        with self._lock:
            if data.score >= self._threshold:
                pose = data.pose.pose
                if data.id not in self._detections.keys():
                    self._detections[data.id] = {}
                self.store_object(data.id, pose)

            if self._debug_enabled: 
                rospy.loginfo(rospy.get_caller_id() + "Detections updated!")
                for (id, pose_lists) in self._detections.items():
                    print(OBJECT_DICT[str(id)])
                    print(len(pose_lists))
            

    def get_distance(self, pose1, pose2): 
        return sqrt((pose2.position.x - pose1.position.x)**2 + 
                    (pose2.position.y - pose1.position.y)**2 +
                    (pose2.position.z - pose1.position.z)**2)  
        
    def store_object(self, id, pose): 
    
        new_detection = (rospy.Time.now(), pose)
        for k, (_, prev_pose) in self._detections[id].items():

            # if an earlier detection of the same object exists, replace item
            if (self.get_distance(pose, prev_pose) < self._distance_threshold):
                self._detections[id][k] = new_detection
                return 
            
        # detection was not yet stored, add a new item 
        self._detections[id][len(self._detections[id])] = new_detection
        return 
    
    def get_detections(self): 
        detections = None 
        with self._lock: 
            detections = self._detections
        return detections
             

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    listener = DetectionSubscriber()
    rospy.loginfo("Detection subscriber node started")
    rospy.spin()
