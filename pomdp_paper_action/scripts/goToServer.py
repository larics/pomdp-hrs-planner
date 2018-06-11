#! /usr/bin/env python

import rospy
import actionlib
import pomdp_paper_action.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import random

class goToAction(object):
    _feedback = pomdp_paper_action.msg.goToFeedback()
    _result = pomdp_paper_action.msg.goToResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, pomdp_paper_action.msg.goToAction, execute_cb=self.execute_cb, auto_start = False)
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
        self.talker(goal)
        rospy.sleep(1)
        self.listener()
        rospy.sleep(1)
        go = goal.goToPositionxy

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
    rospy.init_node('goTo')
    server = goToAction(rospy.get_name())
    rospy.spin()

