#! /usr/bin/env python

import rospy
import actionlib
import pomdp.msg
import random

class OpenLeftAction(object):
    # create messages that are used to publish feedback/result
    _feedback = pomdp.msg.Open_LeftFeedback()
    _result = pomdp.msg.Open_LeftResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pomdp.msg.Open_LeftAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
	#just send random observation
    def execute_cb(self, goal):
        success = True
        # send status
        self._feedback.status='opening left door'

        # publish info to the console for the user
	if goal==0:
		#tiger is left, you opened left door
        	rospy.loginfo('%s: Executing, LOST : opened left door and tiger is left' % (self._action_name))
	else:
		rospy.loginfo('%s: Executing, WON : opened left door and tiger is right' % (self._action_name))
        
        # start executing the action 
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
    rospy.init_node('open_left')
    server = OpenLeftAction(rospy.get_name())
    rospy.spin()
