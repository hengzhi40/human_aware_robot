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
		tmp = []
		for ii in range(0,100):
			tmp.append(0)
		for ii in range(0,100):
			self.socialCost.append(tmp)

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


	# calculate the social cost for points around the human
	def get_social_cost(self):
		# clear the existing cost map
		self.clear_map()

		tx = int(self.humanTransform.transform.translation.x * 10)
		ty = int(self.humanTransform.transform.translation.y * 10)

		# calculate for 5 points (0.5m) before and after the human in either direction
		max_cost = 0
		for ii in range(1,11):
			x = round(ii/10.0 - 0.6, 1)
			for jj in range(1,11):
				y = round(jj/10.0 - 0.6, 1)
				# scale cost by a constant amplitude
				self.socialCost[ty][tx] = 255*self.calculate_gaussian_cost(x,y)	
				max_cost = max(max_cost, self.socialCost[ty][tx])	
		return max_cost # for debugging, remove later
		

	# helper function to clear the map every iteration
	def clear_map(self):
		for ii in range(0, len(self.socialCost)):
			for jj in range(0, len(self.socialCost[0])):
				self.socialCost[ii][jj] = 0


	# calculate the social cost for each point based on bivariate gaussian function
	def calculate_gaussian_cost(self, x, y):
		mean_x = self.humanTransform.transform.translation.x
		mean_y = self.humanTransform.transform.translation.y 
		variance_x = 0.04
		variance_y = 0.0625
		g = (((x-mean_x)**2)/(2*variance_x)) + (((y-mean_y)**2)/(2*variance_y)) 
		p = math.exp(-g)
		return p

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
			# print(m)
			self.go_to_goal(7.5, 7.5, 0.1)

			# publish the velocity message
			self.publisher.publish(self.msg)
			self.rate.sleep()



if __name__ == '__main__':
	try: 
		turtle = Robot()
		turtle.move()
	except rospy.ROSInterruptException: pass
	rospy.spin()