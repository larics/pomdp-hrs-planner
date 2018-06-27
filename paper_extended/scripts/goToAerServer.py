#! /usr/bin/env python

import rospy
import actionlib
import paper_extended.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import random

class goToAerAction(object):
    _feedback = paper_extended.msg.goToAerFeedback()
    _result = paper_extended.msg.goToAerResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, paper_extended.msg.goToAerAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def callback(self, data):
        self._feedback.currentPosition = [round(data.pose.position.x,1), round(data.pose.position.y,1)]

    def listener(self):
        rospy.Subscriber("/vpc_mmcuav/pose", PoseStamped, self.callback)

    def talker(self, goal):
        pub = rospy.Publisher('/vpc_mmcuav/pose_ref', Pose, queue_size=2)
        msg = Pose()
        msg.position.x, msg.position.y = goal.goToPositionxy[0], goal.goToPositionxy[1]
        msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = 1.0, 0.0, 0.0, 0.0, 0.0
        pub.publish(msg)

    def execute_cb(self, goal):
        success = True
	self.listener()
        self.talker(goal)
        go = goal.goToPositionxy
	rospy.sleep(1)

        while not((self._feedback.currentPosition[0]==go[0]) and (self._feedback.currentPosition[1]==go[1])):
            self.talker(goal)
            rospy.sleep(2)
            rospy.loginfo('%s: Driving, position in x: %.1f position in y : %.1f' % (self._action_name, self._feedback.currentPosition[0], self._feedback.currentPosition[1]))
            self._as.publish_feedback(self._feedback)

            # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        if success:
            if ((self._feedback.currentPosition[0]==go[0]) and (self._feedback.currentPosition[1]==go[1])):
                self._result.observation=round(random.uniform(0,1))
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('goToAer')
    server = goToAerAction(rospy.get_name())
    rospy.spin()

