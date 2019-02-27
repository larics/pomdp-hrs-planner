#!/usr/bin/env python
from pomdp_hrs_planner.srv import *
import rospy

def get_new_action_client(obs):
	rospy.wait_for_service('get_new_action')
	try:
		new_action = rospy.ServiceProxy('get_new_action', GetNewAction)
		response = new_action(obs)
		return response.Act
	except rospy.ServiceException, e:
		print "Service call failed: %s" % e


if __name__ == "__main__":
	print("Will do the test now")
	try:
		while True:
			obs_human = raw_input("Enter human observation: ")
			obs_fire = raw_input("Enter fire observation: ")
			action = get_new_action_client([obs_human, obs_fire])
			print("Optimal action is: %s" % action)
			print("------------------------------")
	except KeyboardInterrupt:
		print("Print exiting")
