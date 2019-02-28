#!/usr/bin/env python
from pomdpx_parser import pomdpx_parser as parser
import xml.etree.ElementTree as ET 
import rospy
from std_msgs.msg import String
import numpy as np

num_of_rooms = 6

class RewardCollector:

	def __init__(self):
		model_filename = rospy.get_param("~model")
		root_model = ET.parse(model_filename).getroot()
		self.reward_model, self.states, self.actions, self.state_names = parser.get_reward_model(root_model)
		human_loc = rospy.get_param("~human_loc")
		fire_loc = rospy.get_param("~fire_loc")
		self.state_vectors_human = []
		self.state_vectors_fire = []
		self.total_reward = [0]*num_of_rooms
		for i in range(num_of_rooms):
			state_vector_human = np.zeros(len(self.states[0]))
			state_vector_fire = np.zeros(len(self.states[0]))
			if i+1 == human_loc:
				state_vector_human[1] = 1.0
			else:
				state_vector_human[2] = 1.0
			if i+1 == fire_loc:
				state_vector_fire[1] = 1.0
			else:
				state_vector_fire[2] = 1.0

			self.state_vectors_human.append(state_vector_human)
			self.state_vectors_fire.append(state_vector_fire)


	def callback_room_1(self, action):
		action_index = self.actions[0].index(action.data)
		human_reward = np.sum(self.reward_model[action_index][0]*self.state_vectors_human[0])
		fire_reward = np.sum(self.reward_model[action_index][1]*self.state_vectors_fire[0])
		self.total_reward[0] += (human_reward+fire_reward)
		print(self.total_reward)

	def callback_room_2(self, action):
		action_index = self.actions[0].index(action.data)
		human_reward = np.sum(self.reward_model[action_index][0]*self.state_vectors_human[1])
		fire_reward = np.sum(self.reward_model[action_index][1]*self.state_vectors_fire[1])
		self.total_reward[1] += (human_reward+fire_reward)
		print(self.total_reward)

	def callback_room_3(self, action):
		action_index = self.actions[0].index(action.data)
		human_reward = np.sum(self.reward_model[action_index][0]*self.state_vectors_human[2])
		fire_reward = np.sum(self.reward_model[action_index][1]*self.state_vectors_fire[2])
		self.total_reward[2] += (human_reward+fire_reward)
		print(self.total_reward)

	def callback_room_4(self, action):
		action_index = self.actions[0].index(action.data)
		human_reward = np.sum(self.reward_model[action_index][0]*self.state_vectors_human[3])
		fire_reward = np.sum(self.reward_model[action_index][1]*self.state_vectors_fire[3])
		self.total_reward[3] += (human_reward+fire_reward)
		print(self.total_reward)

	def callback_room_5(self, action):
		action_index = self.actions[0].index(action.data)
		human_reward = np.sum(self.reward_model[action_index][0]*self.state_vectors_human[4])
		fire_reward = np.sum(self.reward_model[action_index][1]*self.state_vectors_fire[4])
		self.total_reward[4] += (human_reward+fire_reward)
		print(self.total_reward)

	def callback_room_6(self, action):
		action_index = self.actions[0].index(action.data)
		human_reward = np.sum(self.reward_model[action_index][0]*self.state_vectors_human[5])
		fire_reward = np.sum(self.reward_model[action_index][1]*self.state_vectors_fire[5])
		self.total_reward[5] += (human_reward+fire_reward)
		print(self.total_reward)

	def subscribe(self):
		rospy.Subscriber("/room1/action", String, self.callback_room_1)
		rospy.Subscriber("/room2/action", String, self.callback_room_2)
		rospy.Subscriber("/room3/action", String, self.callback_room_3)
		rospy.Subscriber("/room4/action", String, self.callback_room_4)
		rospy.Subscriber("/room5/action", String, self.callback_room_5)
		rospy.Subscriber("/room6/action", String, self.callback_room_6)
		rospy.spin()


if __name__ == '__main__':
	rospy.init_node('reward_collector')
	rc = RewardCollector()
	rc.subscribe()

