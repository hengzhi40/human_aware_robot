#!/usr/bin/env python

"""
Script for listening to tf2 and following the leader turtle

Haritha Murali (hm535)
30 September 2018
"""

import rospy
import math
import tf2_ros
import geometry_msgs.msg 
import turtlesim.srv
import turtlesim.msg

class Robot:
	def __init__(self):
		rospy.init_node('robot_listener')

		# get parameters from the parameter server
		self.turtlename = rospy.get_param('turtle', 'turtle2')
		self.human = rospy.get_param('turtle', 'turtle1')

		# spawn service creates multiple turtles
		rospy.wait_for_service('spawn')
		spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
		spawner(1, 1, 0.5*math.pi, self.turtlename)

		# listener for human pose
		self.transformBuffer = tf2_ros.Buffer()
		self.humanTransform = geometry_msgs.msg.TransformStamped()
		listener = tf2_ros.TransformListener(self.transformBuffer)

		# social cost around human
		self.socialCost = []
		# heuristic cost to goal configuration
		self.heuristicCost = []
		# total cost
		self.totalCost = []
		# initialize all costs to 0
		tmp = []
		for ii in range(0,100):
			tmp.append(0)
		for ii in range(0,100):
			self.socialCost.append(tmp[:])
			self.heuristicCost.append(tmp[:])
			self.totalCost.append(tmp[:])

		# subscriber for self pose
		self.subscriber = rospy.Subscriber('%s/pose' % self.turtlename, turtlesim.msg.Pose, self.update_pose)
		self.pose = turtlesim.msg.Pose()

		# publisher for the Twist message
		self.publisher = rospy.Publisher('%s/cmd_vel' % self.turtlename, geometry_msgs.msg.Twist, queue_size=1)
		self.msg = geometry_msgs.msg.Twist()

		self.rate = rospy.Rate(10)

	def update_pose(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x, 3)
		self.pose.y = round(self.pose.y, 3)


	# get human pose in self (robot) frame
	# round the x and y position to prevent "jerky" movement
	def get_human_transform(self):
		self.humanTransform = self.transformBuffer.lookup_transform(self.turtlename, 'turtle1', rospy.Time())
		self.humanTransform.transform.translation.x = round(self.humanTransform.transform.translation.x, 1)
		self.humanTransform.transform.translation.y = round(self.humanTransform.transform.translation.y, 1)


	# helper function to clear the map every iteration
	def clear_map(self, cost_map):
		for ii in range(0, len(cost_map)):
			for jj in range(0, len(cost_map[0])):
				self.socialCost[ii][jj] = 0


	# calculate the social cost for points around the human
	def get_social_cost(self):
		
		# clear the existing cost map
		self.clear_map(self.socialCost)

		max_cost = 0 #for debugging, remove later
		for ii in range(0, len(self.socialCost)):
			y = round(ii/10.0, 2)
			for jj in range(0, len(self.socialCost)):
				x = round(jj/10.0, 2)
				cost = 255.0*self.calculate_gaussian_cost(x, y)
				self.socialCost[ii][jj] = cost
				max_cost = max(max_cost, cost)
		return max_cost # for debugging, remove later
		

	# calculate the social cost for each point based on bivariate gaussian function
	def calculate_gaussian_cost(self, x, y):
		mean_x = self.humanTransform.transform.translation.x
		mean_y = self.humanTransform.transform.translation.y 
		variance_x = 0.0625
		variance_y = 0.0625
		g = (((x-mean_x)**2)/(2*variance_x)) + (((y-mean_y)**2)/(2*variance_y)) 
		p = math.exp(-g)
		return p


	# calculate the heuristic cost for reachable points arond turtle - euclidean distance
	def get_heuristic_cost(self, goal_x, goal_y):
		self.clear_map(self.heuristicCost)

		min_cost = 0 #for debugging
		for ii in range(0, len(self.heuristicCost)):
			y = round(ii/10.0, 2)
			for jj in range(0, len(self.heuristicCost)):
				x = round(jj/10.0, 2)
				distance = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
				self.heuristicCost[jj][ii] = 20.0*distance
				min_cost = min(min_cost, self.heuristicCost[jj][ii])
		return min_cost
		

	def get_total_cost(self):
		self.clear_map(self.totalCost)
		for ii in range(0, len(self.totalCost)):
			for jj in range(0, len(self.totalCost)):
				self.totalCost[ii][jj] = self.socialCost[ii][jj] + self.heuristicCost[ii][jj]

	def search_neighbours(self):
		# for points around turtle pick the nn with lowest cost
		tx = min(max(int(self.pose.x * 10) - 11, 0),78)
		ty = min(max(int(self.pose.y * 10) - 11, 0),78)
		
		min_cost = 1e6
		min_x = tx
		min_y = ty
		for ii in range(1,21):
			x = tx + ii
			for jj in range(1, 21):
				y = ty + jj
				if (self.totalCost[x][y] < min_cost):
					min_cost = self.totalCost[x][y]
					min_x = x 
					min_y = y 
		
		next_x = float(min_y/10.0)
		next_y = float(min_x/10.0)
		return (next_x, next_y, min_cost)


	# follower function from assignment #2
	def follower(self, trans):
		# set angular velocity
		self.msg.angular.z = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
		# set linear velocity
		self.msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)


	# go to specified x and y position within a certain tolerance
	def go_to_goal(self, goal_x, goal_y, tolerance):
		
		distance = math.sqrt((goal_x - self.pose.x)**2 + (goal_y - self.pose.y)**2)
		
		if (distance > tolerance):
			self.msg.linear.x = distance # linear speed is proportional to distance
			self.msg.angular.z = 4 * (math.atan2(goal_y- self.pose.y, goal_x - self.pose.x) - self.pose.theta)
		else:
			self.msg.linear.x = 0
			self.msg.angular.z = 0


	def move(self):
		while not rospy.is_shutdown():

			try:
				self.get_human_transform()
			# exception for error catching
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				self.rate.sleep()
				continue

			# self.follower(self.humanTransform)
			m = self.get_social_cost()
			n = self.get_heuristic_cost(7.5, 7.5)
			self.get_total_cost()
			(next_x, next_y, min_cost) = self.search_neighbours()
			self.go_to_goal(next_x, next_y, 0.01)
			# self.go_to_goal(7.5, 7.5, 0.01)
			print(min_cost)

			# publish the velocity message
			self.publisher.publish(self.msg)
			self.rate.sleep()



if __name__ == '__main__':
	try: 
		turtle = Robot()
		turtle.move()
	except rospy.ROSInterruptException: pass
	rospy.spin()