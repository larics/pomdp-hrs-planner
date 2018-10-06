#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import numpy as np
import random
from pomdp.srv import *
import actionlib
import pomdp.msg

def Calculating_pomdp_client(observation):
    rospy.wait_for_service('Calculating_pomdp')
    try:
        Calculating_pomdp = rospy.ServiceProxy('Calculating_pomdp', CalculatingNewAction)
        resp1 = Calculating_pomdp(observation)
        return resp1.OptAct
    except rospy.ServiceException, e:
        print ("Service call failed: %s",e)

def Observe_client(tigerPos):
    client = actionlib.SimpleActionClient('observe', pomdp.msg.ObserveAction)
    client.wait_for_server()
    goal = pomdp.msg.ObserveGoal(tigerPosition = tigerPos)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def Open_right_client(tigerPos):
    client = actionlib.SimpleActionClient('open_right', pomdp.msg.Open_RightAction)
    client.wait_for_server()
    goal = pomdp.msg.Open_RightGoal(openRightDoor = tigerPos)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

def Open_left_client(tigerPos):
    client = actionlib.SimpleActionClient('open_left', pomdp.msg.Open_LeftAction)
    client.wait_for_server()
    goal = pomdp.msg.Open_LeftGoal(openLeftDoor = tigerPos)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == "__main__":
   #initialization of pomdp calculator
   observation=-1
   # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
   rospy.init_node('Master_client')

   while 1==1:
       #random variable that decides is tiger left or right
       #if tigerPosition==0 tiger is left, and if tigerPosition==1 tiger is right
       tigerPos = int(round (random.uniform(0,1)))
       print("New tiger position : ", (tigerPos))
       loop=True

       while loop==True:
           action = Calculating_pomdp_client(observation)
           print("Action : ", action)
           if action==0:
              #do action listen
              try:
                  obs = Observe_client(tigerPos)
                  observation = obs.observation
                  print("Observation ",(observation))
              except rospy.ROSInterruptException:
                  print("program interrupted before completion", file=sys.stderr)

           if action==1:
               #tiger is right
              try:
                  obs = Open_left_client(tigerPos)
                  observation = obs.observation
                  print("Observation :",(observation))
              except rospy.ROSInterruptException:
                  print("program interrupted before completion", file=sys.stderr)
              loop=False

           if action==2:
               #tiger is left
               try:
                   obs = Open_right_client(tigerPos)
                   observation = obs.observation
                   print("Obsevation :",(observation))
               except rospy.ROSInterruptException:
                   print("program interrupted before completion", file=sys.stderr)
               loop=False


