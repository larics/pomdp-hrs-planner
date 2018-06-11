#!/usr/bin/env python
#simulates people talk

import rospy
from pomdp_paper_action.msg import *
import random

def beacon(coord, rang):
    pub = rospy.Publisher('/beacon', Senzor, queue_size=1)

    rospy.init_node('beacon', anonymous=True)
    msg = Senzor()
    msg.radius = rang
    msg.positionx, msg.positiony =  coord[0], coord[1]
    i=0
    while not rospy.is_shutdown():
	if (i%7==0):
	     msg.obser = 0
	else:
	     msg.obser = 1
	i=i+1
	if (i==100):
	    i=0
        pub.publish(msg)

if __name__ == '__main__':
    rang = 3.0
    fullRoom = int(round (random.uniform(-0.5,5.5)))
    print("Beacon is in the room : ", (fullRoom))

   #room number(0-5) [Xmin,Xmax,Ymin,Ymax]
    roomMatrix = [[-3.0,1,0,5],[-3,1,5,10],[-3,1,10,15],[6,10,10,15],[6,10,5,10],[6,10,0,5]]

    #beacon position -->in the middle of the room
    beaconPosition = [roomMatrix[fullRoom][0]+0.5*(roomMatrix[fullRoom][1]-roomMatrix[fullRoom][0]),roomMatrix[fullRoom][2]+ 0.5*(roomMatrix[fullRoom][3]-roomMatrix[fullRoom][2])]
    try:
        beacon(beaconPosition, rang)
    except rospy.ROSInterruptException:
        pass
