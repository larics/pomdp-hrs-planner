#! /usr/bin/env python

import rospy
import actionlib
import pomdp_paper_action.msg
import random
import math
from pomdp_paper_action.msg import *

class ListenAction(object):
    # create messages that are used to publish feedback/result
    _feedback = pomdp_paper_action.msg.ListenFeedback()
    _result = pomdp_paper_action.msg.ListenResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pomdp_paper_action.msg.ListenAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def callback(self, data):
        self.rang = round(data.radius,1)
	self.beaconx = round(data.positionx,1)
	self.beacony =  round(data.positiony,1)
	self.observation = data.obser

    def listener(self):
        rospy.Subscriber("/beacon", Senzor, self.callback)
      
	#goal= [current drone position]]
    def execute_cb(self, goal):
        success = True
        self._feedback.status='listening'
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, listening' % (self._action_name))
        self.listener()

        #calculate distance between drone and beacon
        distance = round(math.sqrt( (goal.input[1]-self.beacony)**2 + (goal.input[0]-self.beaconx)**2),1)
        
        # start executing the action 
        if (distance>self.rang):
                self._result.observation = 0
        else : 	
		self._result.observation = self.observation

            # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
		rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
          
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('listen')
    server = ListenAction(rospy.get_name())
    rospy.spin()
