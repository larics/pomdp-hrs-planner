#! /usr/bin/env python

import rospy
import actionlib
import pomdp.msg
import random

class ObserveAction(object):
    # create messages that are used to publish feedback/result
    _feedback = pomdp.msg.ObserveFeedback()
    _result = pomdp.msg.ObserveResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pomdp.msg.ObserveAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
	#if goal==0 tiger is left, otherwise right
    def execute_cb(self, goal):
        success = True
        # send status
        self._feedback.status='listening'
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, listening' % (self._action_name))
        
        # start executing the action 
	#in 85% of the cases send good observation	
	if int(random.uniform(0, 100))<85:
		self._result.observation=goal.tigerPosition
	else:
		self._result.observation=int(not goal.tigerPosition)

            # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
		rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('observe')
    server = ObserveAction(rospy.get_name())
    rospy.spin()
