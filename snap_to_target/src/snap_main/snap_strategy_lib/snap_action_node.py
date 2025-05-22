#!/usr/bin/env python3
import rospy 
import actionlib
import string 
from abc import ABC, abstractmethod
from snap_to_target.msg import SnapAction, SnapFeedback, SnapResult 

class SnapStrategy(ABC):
    """
    Interface for snap to target strategy  
    """
    def __init__(self, target_type : str) -> None:
        super().__init__()
        self._target_type = target_type

    @property
    def target_type(self) -> str: 
        """
        Reference to snap strategy target type 

        Returns:
            str: Target type
        """
        return self._target_type

    @abstractmethod
    def reset(self): 
        pass

    @abstractmethod
    def snap_to_target(self, goal):
        pass 


class SnapActionNode: 
    """
    Defines the interface for snap to target action servers 
    """
    def __init__(self, snap_strategy: SnapStrategy, action_name: string, 
                 num_attempts: int) -> None: 

        self._feedback = SnapFeedback() 
        self._result = SnapResult() 

        self._snap_strategy = snap_strategy
        self._action_name = action_name
        self._num_attempts = num_attempts
        
        self._server = None     # actionlib server 
        
    def action_callback(self, goal) -> None: 
        """
        Action callback. Attempts to find a target to snap, using the 
        defined strategy. 

        Args:
            goal (SnapActionGoal): Defines the target type. 
        """

        # poll with 1 second interval
        for attempt in range(self._num_attempts):

            # action is cancelled
            if self._server.is_preempt_requested(): 
                rospy.loginfo("[%s] Pre-empted", self._action_name)
                self._server.set_preempted()
                return self._result
            
            # log message to ros info 
            msg = f"[{self._action_name}] Attempt no.{str(attempt)} to snap to target"
            rospy.loginfo(msg)

            # publish feedback to the client 
            self._feedback.feedback_msg = msg
            self._server.publish_feedback(self._feedback)

            # attempt snap to target 
            target_id, target_pose, is_group = self._snap_strategy.snap_to_target(goal)
            
            # if target is verified, action is successful 
            if target_id is not None and target_pose is not None: 
                self._result.target_id = target_id
                self._result.target_pose = target_pose
                self._result.is_group = is_group
                
                rospy.loginfo(f"[{self._action_name}] Snapping to {self._snap_strategy.target_type} with id {target_id} succeeded.")
                self._server.set_succeeded(self._result)
                self._snap_strategy.reset()
                return self._result
            else: 
                self._snap_strategy.reset()
                rospy.sleep(1)

        rospy.loginfo(f"No {self._snap_strategy.target_type} found.")
        self._snap_strategy.reset()
        self._server.set_aborted()


    def start_node(self): 
        """
        Starts the action server with the given action name
        """
        self._server = actionlib.SimpleActionServer(self._action_name,
                                                    SnapAction, 
                                                    execute_cb = 
                                                    self.action_callback,
                                                    auto_start=False)
        rospy.loginfo(f"{self._action_name} server started")
        self._server.start() 


