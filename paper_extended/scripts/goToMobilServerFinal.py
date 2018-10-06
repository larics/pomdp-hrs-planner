#! /usr/bin/env python
#moving mobile platform and UAV at the same time

import rospy
import actionlib
import paper_extended.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import random
import numpy as np

class goToMobilAction(object):
    _feedback = paper_extended.msg.goToMobilFeedback()
    _result = paper_extended.msg.goToMobilResult()

    def __init__(self, name):
        self._action_name = name
        self.goalMobile = [0.0,0.0, 0.0]
        self._currentPosition = [0.0, 0.0, 0.0]
        self._as = actionlib.SimpleActionServer(self._action_name, paper_extended.msg.goToMobilAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def callbackDrone(self, dataD):
        self._feedback.currentPosition = [round(dataD.pose.position.x,1), round(dataD.pose.position.y,1)]

    def listenerDrone(self):
        rospy.Subscriber("/vpc_mmcuav/pose", PoseStamped, self.callbackDrone)

    def callbackMobile(self, dataM):
        self._currentPosition = [round(dataM.pose.pose.position.x,1), round(dataM.pose.pose.position.y,1), round(dataM.pose.pose.orientation.z,1)]
        self.Q = [round(dataM.pose.pose.orientation.w,1), round(dataM.pose.pose.orientation.x,1), round(dataM.pose.pose.orientation.y,1), round(dataM.pose.pose.orientation.z,1)]

    def listenerMobile(self):
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.callbackMobile)

    def talkerPositionAer(self):
        pub = rospy.Publisher('/vpc_mmcuav/pose_ref', Pose, queue_size=2)
        msg = Pose()
        msg.position.x, msg.position.y = self._currentPosition[0], self._currentPosition[1]
        msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = 0.4, 0.0, 0.0, 0.0, 0.0
        pub.publish(msg)

    def CalculateGoal(self, goal):
        deltax = -self._currentPosition[0]+ goal.goToPositionxyVelocity[0]
        deltay = -self._currentPosition[1]+ goal.goToPositionxyVelocity[1]
        udaljenost = np.sqrt(np.square(deltax)+np.square(deltay))
        wantedAngle = round(np.arctan2(deltay, deltax,),2)
        #limit the velocity to stay in controll of the vehicle and acc
        if (abs(udaljenost)<=0.1):
            self.goalMobile[1] = 0.0
            self.goalMobile[0] = 0.0
        if (abs(udaljenost)<=0.5):
            self.goalMobile[0] = udaljenost*np.cos(wantedAngle)*0.5
            self.goalMobile[1] = np.sin(wantedAngle)*udaljenost*0.5
        if (abs(udaljenost)>0.5):
            self.goalMobile[0] = np.abs(0.5*np.cos(wantedAngle))*np.sign(deltax)
            self.goalMobile[1] = np.abs(0.5*np.sin(wantedAngle))*np.sign(deltay)

    def talkerPositionMobil(self, goal):
        self.CalculateGoal(goal)
        #publish data
        pub = rospy.Publisher('/spur/cmd_vel', Twist, queue_size=2)
        msg = Twist()
        msg.linear.x, msg.linear.y  = self.goalMobile[0], self.goalMobile[1]
        #msg.angular.z = 0.0
        msg.linear.z, msg.angular.x, msg.angular.y = 0.0, 0.0, 0.0
        pub.publish(msg)
        rospy.sleep(1)

    def execute_cb(self, goal):
        success = True
        self.listenerDrone()
        self.listenerMobile()
        rospy.sleep(1)
        self.talkerPositionMobil(goal)
        self.talkerPositionAer()
        go = goal.goToPositionxyVelocity

        while not((self._feedback.currentPosition[0]==go[0]) and (self._feedback.currentPosition[1]==go[1]) and (self._currentPosition[0]==go[0]) and (self._currentPosition[1]==go[1])):
            if not((self._feedback.currentPosition[0]==go[0]) and (self._feedback.currentPosition[1]==go[1])):
                self.talkerPositionAer()
                #rospy.loginfo('%s: Driving drone, position in x: %.1f position in y : %.1f' % (self._action_name, self._feedback.currentPosition[0], self._feedback.currentPosition[1]))
                self._as.publish_feedback(self._feedback)
            if not((self._currentPosition[0]==go[0]) and (self._currentPosition[1]==go[1])):
                self.talkerPositionMobil(goal)
                #rospy.loginfo('%s: Driving mobile, position in x: %.1f position in y : %.1f' % (self._action_name, self._currentPosition[0], self._currentPosition[1]))

            # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False

        if success:
            if ((self._feedback.currentPosition[0]==go[0]) and (self._feedback.currentPosition[1]==go[1]) and (self._currentPosition[0]==go[0]) and (self._currentPosition[1]==go[1])):
                self._result.observation=round(random.uniform(0,1))
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('goToMobil')
    server = goToMobilAction(rospy.get_name())
    rospy.spin()


