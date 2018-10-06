#! /usr/bin/env python

import rospy
import actionlib
import pomdp.msg
import random

class OpenRightAction(object):
    # create messages that are used to publish feedback/result
    _feedback = pomdp.msg.Open_RightFeedback()
    _result = pomdp.msg.Open_RightResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pomdp.msg.Open_RightAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        #param goal:int, represents tigers position
        success = True
        # send status
        self._feedback.status='opening right door'

        # publish info to the console for the user
        if goal==1:
            #tiger is right, you opened right door
        	rospy.loginfo('%s: Executing, LOST : opened right door and tiger is right' % (self._action_name))
        else:
            #tiger is left, you opened right door
            rospy.loginfo('%s: Executing, WON : opened right door and tiger is left' % (self._action_name))

        #just send random observation
	    self._result.observation = round (random.uniform(0,1))

            # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('open_right')
    server = OpenRightAction(rospy.get_name())
    rospy.spin()
