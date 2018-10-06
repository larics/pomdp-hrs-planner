#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
import numpy as np
import random
from paper_extended.srv import *
import actionlib
import paper_extended.msg
from paper_extended.msg import *

def Calculating_pomdp_client(observation):
    rospy.wait_for_service('Calculating_pomdp')
    try:
        Calculating_pomdp = rospy.ServiceProxy('Calculating_pomdp', CalculatingNewAction)
        resp1 = Calculating_pomdp(observation)
        return resp1.OptAct
    except rospy.ServiceException, e:
        print ("Service call failed: %s",e)

#Listening client
def Listen_client(ulaz):
    client = actionlib.SimpleActionClient('listen', paper_extended.msg.ListenAction)
    client.wait_for_server()
    goal = paper_extended.msg.ListenGoal(input = ulaz)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

#goToMobil client
def goToMobil_client(coord):
    client = actionlib.SimpleActionClient('goToMobil', paper_extended.msg.goToMobilAction)
    client.wait_for_server()
    goal = paper_extended.msg.goToMobilGoal(goToPositionxyVelocity = coord)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

#goToAer client
def goToAer_client(coord):
    client = actionlib.SimpleActionClient('goToAer', paper_extended.msg.goToAerAction)
    client.wait_for_server()
    goal = paper_extended.msg.goToAerGoal(goToPositionxy = coord)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == "__main__":
   #intialization
   room = [0,0,0,0,0,0]
   observation=-1
   roomIndex=0
   roomMatrix = [[-3.0,1,0,5],[-3,1,5,10],[-3,1,10,15],[6,10,10,15],[6,10,5,10],[6,10,0,5]]
   #position for listening
   roomListening = [[2,2.5],[2,7.5],[2,12.5],[5,12.5],[5,7.5],[5,2.5]]
   startPosition = [3.5,-1]
   roomEnter = [[2,2.5],[2,7.5],[2,12.5],[5,12.5],[5,7.5],[5,2.5]]
   #enter room position
   for i in range(6):
       roomEnter[i] = [roomMatrix[i][0]+0.5*(roomMatrix[i][1]-roomMatrix[i][0]),roomMatrix[i][2]+ 0.5*(roomMatrix[i][3]-roomMatrix[i][2])]

   rospy.init_node('Master_client')
  
   #send drone and mobile robot to the start position
   result = goToMobil_client(startPosition)
   print("Dosao sam na pocetnu poziciju", result)
   rospy.sleep(5)
   result = goToMobil_client(roomListening[0])
   print("Dosao sam na prvu poziciju za slusanje", result)

   while roomIndex<6:
       action = Calculating_pomdp_client(observation)
       print("Action: ", action)

       #do action observe
       if action==2:
           try:
               obs = Listen_client([roomListening[roomIndex][0], roomListening[roomIndex][1]])
               observation = obs.observation
               print("Observation ",(observation))
           except rospy.ROSInterruptException:
               print("program interrupted before completion", file=sys.stderr)
           rospy.sleep(1)

           #proceed action --> goTo next listening position
       if action==3:
           #mark that the room is full
           room[roomIndex] = 1
           roomIndex = roomIndex+1
           #check if you came to the last room
           if roomIndex>5:
              break
           try:
              obs = goToMobil_client(roomListening[roomIndex])
              observation = obs.observation
              print("Observation :",(observation))
           except rospy.ROSInterruptException:
              print("program interrupted before completion", file=sys.stderr)
           rospy.sleep(1)

           #enter and proceed Mobile
       if action==0:
           try:
               result = goToMobil_client(roomEnter[roomIndex])
               rospy.sleep(1)
               result = goToMobil_client(roomListening[roomIndex])
               rospy.sleep(1)
               roomIndex = roomIndex+1
               #check if you came to the last room
               if roomIndex>5:
                   break
               obs = goToMobil_client(roomListening[roomIndex])
               observation = obs.observation
               print("Obsevation :",(observation))
           except rospy.ROSInterruptException:
               print("program interrupted before completion", file=sys.stderr)
           rospy.sleep(1)

           #enter and proceed Aer
       if action==1:
           try:
               result = goToAer_client(roomEnter[roomIndex])
               rospy.sleep(1)
               result = goToAer_client(roomListening[roomIndex])
               rospy.sleep(1)
               roomIndex = roomIndex+1
               #check if you came to the last room
               if roomIndex>5:
                   break
               obs = goToMobil_client(roomListening[roomIndex])
               observation = obs.observation
               print("Obsevation :",(observation))
           except rospy.ROSInterruptException:
               print("program interrupted before completion", file=sys.stderr)
           rospy.sleep(1)

   #go back to the start position
   result = goToMobil_client(startPosition)





