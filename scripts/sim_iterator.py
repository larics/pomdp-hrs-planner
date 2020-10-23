#!/usr/bin/env python
import rospy
import roslaunch
from std_msgs.msg import String
from consensus_ros.msg import BeliefStamped
import time
import csv

class belief_subscriber():
	def __init__(self, init_belief, name):
		rospy.Subscriber('/%s/belief' % name, BeliefStamped, self.belief_cb)
		self.belief = init_belief

	def belief_cb(self, data):
		self.belief = [ i for i in data.belief.data]

class action_subscriber():
	def __init__(self, name):
		self.got_new_action = False
		rospy.Subscriber('/%s/action' % name, String, self.action_cb)
		self.action = " "

	def action_cb(self, msg):
		self.action = msg.data

class observation_subscriber():
	def __init__(self, name):
		self.got_new_action = False
		rospy.Subscriber('/%s/observation' % name, String, self.observation_cb)
		self.observation = " "

	def observation_cb(self, msg):
		self.observation = msg.data

class SimIterator():
	def __init__(self, uuid):
		self.team = ['UAV1', 'UAV2', 'UAV3']
		init_belief = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

		self.writeout_data = []
		for team in self.team:
			self.writeout_data.append([['t', 'action', 'observation', 'belief']])

		self.t_start = rospy.get_time()
		# team_config = rospy.get_param("/team_config_file")
		# with open(team_config, 'r') as yaml_stream:
		# 	try:
		# 		team = yaml.load(yaml_stream, Loader=yaml.CLoader)
		# 	except yaml.YAMLError as e:
		# 		print(e)
		self.uuid = uuid
		roslaunch.configure_logging(uuid)
		self.act_subsribers = []
		self.obs_subsribers = []
		self.belief_subsribers = []
		for agent in self.team:
			self.act_subsribers.append(action_subscriber(agent))
			self.obs_subsribers.append(observation_subscriber(agent))
			self.belief_subsribers.append(belief_subscriber(init_belief, agent))

	def is_all_end(self):
		val = True
		for subscriber in self.act_subsribers:
			if subscriber.action != 'end':
				val =  False
				break
		return val

	def reset_stuff(self):
		for subscriber in self.act_subsribers:
			subscriber.action = " "
		for subscriber in self.obs_subsribers:
			subscriber.observation = " "
		for subscriber in self.belief_subsribers:
			subscriber.belief = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		self.writeout_data = []
		for team in self.team:
			self.writeout_data.append([['t', 'action', 'observation', 'belief']])


	def get_data(self):
		t = rospy.get_time() - self.t_start
		for i in range(len(self.team)):
			row = [t, self.act_subsribers[i].action, self.obs_subsribers[i].observation]
			for value in self.belief_subsribers[i].belief:
				row.append(value)
			self.writeout_data[i].append(row)

	def start(self):
		rate = rospy.Rate(10)
		rooms = ["room1", "room2", "room3", "room4", "room5", "room6", "room7", "room8", "room9", "room10"]
		room_count = 0
		iter_count = 0
		while not rospy.is_shutdown():
			if room_count >=10:
				break
			# launch_sim = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/frano/devel/pomdp_hrs_ws/src/pomdp-hrs-planner/launch/multiple_agents_sim.launch"])
			cli_args = ['/home/frano/devel/pomdp_hrs_ws/src/pomdp-hrs-planner/launch/multiple_agents_sim.launch','fire_room:=%s' % rooms[room_count]]
			roslaunch_args = cli_args[1:]
			roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
			launch_sim = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)

			# cli_args = ['pomdp_hrs_ws', 'multiple_agents_sim.launch', 'fire_room:=%s' % rooms[room_count]]
			# print(cli_args)
			# roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)
			# roslaunch_args = cli_args[2:]
			# launch_sim = roslaunch.parent.ROSLaunchParent(self.uuid, (roslaunch_file, roslaunch_args))
			launch_sim.start()
			self.t_start = rospy.get_time()
			while not self.is_all_end():
				self.get_data()
				rate.sleep()
			print("Found all end")
			time.sleep(30)
			launch_sim.shutdown()
			print('Terminating, waiting')
			time.sleep(10)
			print('Writing data')
			for i in range(len(self.team)):
				writeout_filename = '/home/frano/devel/pomdp_hrs_ws/src/pomdp-hrs-planner/data/%s_%s_%s.csv' % (self.team[i], rooms[room_count], iter_count)
				with open(writeout_filename, 'wb') as csvfile:
					writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
					for row in self.writeout_data[i][1:]:
						writer.writerow(row)
			iter_count +=1
			if iter_count >= 10:
				room_count +=1
				iter_count = 0
				
			self.reset_stuff()



if __name__ == "__main__":
	rospy.init_node("Simulation_iterator")
	sim_iter = SimIterator(roslaunch.rlutil.get_or_generate_uuid(None, False))
	sim_iter.start()



