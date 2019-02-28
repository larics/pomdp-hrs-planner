#!/usr/bin/env python
from pomdpx_parser import pomdpx_parser as parser
import numpy as np 
import xml.etree.ElementTree as ET 
import copy
from pomdp_hrs_planner.srv import *
from std_msgs.msg import String
import rospy

class CopyPOMDP:

	def __init__(self, pomdp):
		self.policy_vectors = pomdp.policy_vectors[:]
		self.optimal_actions = pomdp.optimal_actions[:]
		self.belief = pomdp.belief[:]
		self.transition_probs = pomdp.transition_probs[:]
		self.observation_probs = pomdp.observation_probs[:]
		self.actions = pomdp.actions[:]
		self.observations = pomdp.observations[:]

	def unpack_belief(self):
		unpacked_belief = self.belief[0]
		if len(self.belief) > 1:
			for i in range(1, len(self.belief)):
				to_calc = unpacked_belief[:]
				unpacked_belief = []
				for j in range(len(to_calc)):
					for k in range(len(self.belief[i])):
						unpacked_belief.append(to_calc[j]*self.belief[i][k])
		return unpacked_belief

	def get_optimal_action(self):
		belief = self.unpack_belief()
		max_value = 0
		max_index = 0
		index = 0
		for m in self.policy_vectors:
			if np.dot(belief, np.transpose(m)) > max_value:
				max_value = np.dot(belief, np.transpose(m))
				max_index = index
			index += 1
		return self.optimal_actions[max_index]

	def update_belief(self, action, observation):
		for i in range(len(self.belief)):
			T = self.transition_probs[i][action]
			O = self.observation_probs[i][action][:, self.observations[i].index(observation[i])]
			next_state_prior = np.dot(np.transpose(T), self.belief[i])
			if np.count_nonzero(next_state_prior) == 1:
				self.belief[i] = next_state_prior
			else:
				self.belief[i] = O * next_state_prior

			if np.linalg.norm(self.belief[i]) == 0:
				self.belief[i] = next_state_prior
			self.belief[i] /= np.sum(self.belief[i])

		return self.belief



class POMDP:

	def __init__(self):
		self.model_filename = rospy.get_param("~model")
		self.policy_filename = rospy.get_param("~policy")
		root_model = ET.parse(self.model_filename).getroot()
		root_policy = ET.parse(self.policy_filename).getroot()
		self.description, self.discount, self.states, self.actions, self.observations, self.state_names = parser.get_general_info(root_model)
		self.policy_vectors, self.optimal_actions = parser.import_policy(root_policy)
		self.belief = parser.get_initial_belief(root_model)
		self.transition_probs = parser.get_matrix('StateTransitionFunction', root_model)
		self.observation_probs = parser.get_matrix('ObsFunction', root_model)
		self.last_action = None
		self.pub = rospy.Publisher('action', String, queue_size=1)

	def unpack_belief(self):
		unpacked_belief = self.belief[0]
		if len(self.belief) > 1:
			for i in range(1, len(self.belief)):
				to_calc = unpacked_belief[:]
				unpacked_belief = []
				for j in range(len(to_calc)):
					for k in range(len(self.belief[i])):
						unpacked_belief.append(to_calc[j]*self.belief[i][k])
		return unpacked_belief

	def get_optimal_action(self):
		belief = self.unpack_belief()
		max_value = 0
		max_index = 0
		index = 0
		for m in self.policy_vectors:
			if np.dot(belief, np.transpose(m)) > max_value:
				max_value = np.dot(belief, np.transpose(m))
				max_index = index
			index += 1
		return self.optimal_actions[max_index]

	def update_belief(self, action, observation):
		for i in range(len(self.belief)):
			T = self.transition_probs[i][action]
			O = self.observation_probs[i][action][:, self.observations[i].index(observation[i])]
			next_state_prior = np.dot(np.transpose(T), self.belief[i])
			if np.count_nonzero(next_state_prior) == 1:
				self.belief[i] = next_state_prior
			else:
				self.belief[i] = O * next_state_prior

			if np.linalg.norm(self.belief[i]) == 0:
				self.belief[i] = next_state_prior
			self.belief[i] /= np.sum(self.belief[i])

		return self.belief

	def predict_most_likely_action(self, action):
		pomdp_c = CopyPOMDP(self)
		obs = [-1]*len(pomdp_c.belief)
		for i in range(len(pomdp_c.belief)):
			T = pomdp_c.transition_probs[i][action]
			next_state_prior = np.dot(np.transpose(T), pomdp_c.belief[i])
			probs = [0]*len(pomdp_c.observations[i])
			for j in range(len(pomdp_c.observations[i])):
				O = pomdp_c.observation_probs[i][action][:, j]
				probs[j] = np.sum(O*next_state_prior)

			obs[i] = pomdp_c.observations[i][probs.index(max(probs))]
		optA = self.actions[0].index(action)
		pomdp_c.update_belief(pomdp_c.actions[0][int(optA)], obs)
		return pomdp_c.get_optimal_action()

	def handle_get_new_action(self, req):
		if self.last_action:
			self.update_belief(self.last_action, req.Obs)
		ActNum = self.get_optimal_action()
		self.last_action = self.actions[0][ActNum]
		self.pub.publish(self.last_action)
		return GetNewActionResponse(self.last_action)

	def handle_predict_next_action(self, req):
		if self.last_action:
			print("Predict most likely action with last_action %s" % self.last_action)
			ml_action = self.predict_most_likely_action(self.last_action)
		else:
			print("Get next action without last action")
			ml_action = self.get_optimal_action()
		return PredictNextActionResponse(self.actions[0][ml_action])

	def service_server(self):
		ser1 = rospy.Service('get_new_action', GetNewAction, self.handle_get_new_action)
		ser2 = rospy.Service('predict_next_action', PredictNextAction, self.handle_predict_next_action)
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('get_new_action_server')
	pomdp = POMDP()
	pomdp.service_server()