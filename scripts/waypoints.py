#!/usr/bin/env python

import copy, time, yaml

# Ros imports
import rospy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped
from std_msgs.msg import String

class SingleAgentBuildingTour():

	def __init__(self):
		locations_config = rospy.get_param("/room_coordinates_file")
		with open(locations_config, 'r') as yaml_stream:
			try:
				self.locations = yaml.load(yaml_stream, Loader=yaml.CLoader)
			except yaml.YAMLError as e:
				print(e)
		self.action = 'None'
		self.previous_action = 'None'
		self.trajectory = []
		
	def start(self):
		self.starting_pose = rospy.wait_for_message("pose", PoseStamped) #only need starting pose
		self.locations['start'] = [self.starting_pose.pose.position.x, self.starting_pose.pose.position.y, self.starting_pose.pose.position.z, 0.0]
		rospy.Subscriber("action", String, self.action_callback)
		self.trajectory_pub = rospy.Publisher('multi_dof_trajectory', MultiDOFJointTrajectory, queue_size=5)
		rospy.spin()
		
	def action_callback(self, msg):
		self.previous_action = self.action
		self.action = msg.data
		self.define_trajectory_points()
		time.sleep(10)
		self.trajectory_pub.publish(self.trajectory)
			

	def define_trajectory_points(self):
		if self.action != 'end':
			if self.previous_action !='None':
				action_no = self.action.strip('scout')
				previous_action_no = self.previous_action.strip('scout')
				x = [self.locations['room' + previous_action_no + '_inside'][0], self.locations['room' + previous_action_no + '_outside'][0], self.locations['room' + action_no + '_outside'][0], 						self.locations['room' + action_no + '_inside'][0]]
				y = [self.locations['room' + previous_action_no + '_inside'][1], self.locations['room' + previous_action_no + '_outside'][1], self.locations['room' + action_no + '_outside'][1], 						self.locations['room' + action_no + '_inside'][1]]
				z = [self.locations['room' + previous_action_no + '_inside'][2], self.locations['room' + previous_action_no + '_outside'][2], self.locations['room' + action_no + '_outside'][2], 						self.locations['room' + action_no + '_inside'][2]]
				yaw = [self.locations['room' + previous_action_no + '_inside'][3], self.locations['room' + previous_action_no + '_outside'][3], self.locations['room' + action_no + '_outside'][3], 					self.locations['room' + action_no + '_inside'][3]]
			else:
				action_no = self.action.strip('scout')
				x = [self.locations['start'][0], self.locations['room' + action_no + '_outside'][0], self.locations['room' + action_no + '_inside'][0]]
				y = [self.locations['start'][1], self.locations['room' + action_no + '_outside'][1], self.locations['room' + action_no + '_inside'][1]]
				z = [self.locations['start'][2], self.locations['room' + action_no + '_outside'][2], self.locations['room' + action_no + '_inside'][2]]
				yaw = [self.locations['start'][3], self.locations['room' + action_no + '_outside'][3], self.locations['room' + action_no + '_inside'][3]]
					
		else:
			if self.previous_action !='None':
				previous_action_no = self.previous_action.strip('scout')
				x = [self.locations['room' + previous_action_no + '_inside'][0], self.locations['room' + previous_action_no + '_outside'][0], self.locations['start'][0]]
				y = [self.locations['room' + previous_action_no + '_inside'][1], self.locations['room' + previous_action_no + '_outside'][1], self.locations['start'][1]]
				z = [self.locations['room' + previous_action_no + '_inside'][2], self.locations['room' + previous_action_no + '_outside'][2], self.locations['start'][2]]
				yaw = [self.locations['room' + previous_action_no + '_inside'][3], self.locations['room' + previous_action_no + '_outside'][3], self.locations['start'][3]]
				self.action = 'None'
				self.previous_action = 'None'	
		self.request_trajectory(x,y,z,yaw)
		
				
	def request_trajectory(self, x ,y ,z , yaw):
		rospy.wait_for_service("generate_toppra_trajectory")
		try:
			request_trajectory_service = rospy.ServiceProxy("generate_toppra_trajectory", GenerateTrajectory)
			request = GenerateTrajectoryRequest()
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e	
		
		waypoint = JointTrajectoryPoint()
		for i in range(0, len(x)):
			waypoint.positions = [x[i], y[i], z[i], yaw[i]]
			if i==0:
				waypoint.velocities = [1, 1, 1, 1]
				waypoint.accelerations = [0.6, 0.6, 0.6, 1]
			request.waypoints.points.append(copy.deepcopy(waypoint))
			request.waypoints.points.append(copy.deepcopy(waypoint))
		request.waypoints.joint_names = ["x", "y", "z", "yaw"]
		request.sampling_frequency = 100.0
		request.n_gridpoints = 500
		request.plot = False
		response = request_trajectory_service(request)
        	
		joint_trajectory = response.trajectory
		self.trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
        	
    
	def JointTrajectory2MultiDofTrajectory(self, joint_trajectory):
        	multi_dof_trajectory = MultiDOFJointTrajectory()

        	for i in range(0, len(joint_trajectory.points)):
			temp_point = MultiDOFJointTrajectoryPoint()
			temp_transform = Transform()
			temp_transform.translation.x = joint_trajectory.points[i].positions[0]
			temp_transform.translation.y = joint_trajectory.points[i].positions[1]
			temp_transform.translation.z = joint_trajectory.points[i].positions[2]
			temp_transform.rotation.w = 1.0

			temp_vel = Twist()
			temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
			temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
			temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

			temp_acc = Twist()
			temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
			temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
			temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

			temp_point.transforms.append(temp_transform)
			temp_point.velocities.append(temp_vel)
			temp_point.accelerations.append(temp_acc)
			temp_point.time_from_start = joint_trajectory.points[i].time_from_start

			multi_dof_trajectory.points.append(temp_point)

		return multi_dof_trajectory
				


if __name__ == "__main__":
    rospy.init_node("Building_Tour")
    agent1 = SingleAgentBuildingTour()
    agent1.start()
    
