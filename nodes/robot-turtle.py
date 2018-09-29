#!/usr/bin/env python

"""
Script for listening to tf2 and following the leader turtle

Haritha Murali (hm535)
21 September 2018
"""

import rospy
import math
import tf2_ros
import geometry_msgs.msg 
import turtlesim.srv
import turtlesim.msg

class followerTurtle:
	def __init__(self):
		rospy.init_node('turtle_listener')

		# get parameters from the parameter server
		self.turtlename = rospy.get_param('turtle', 'turtle2')

		# spawn service creates multiple turtles
		rospy.wait_for_service('spawn')
		spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
		spawner(1, 1, 0.5*math.pi, self.turtlename)

		# listener for human pose
		self.transformBuffer = tf2_ros.Buffer()
		listener = tf2_ros.TransformListener(self.transformBuffer)

		# subscriber for self pose
		self.subscriber = rospy.Subscriber('%s/pose' % self.turtlename, turtlesim.msg.Pose, self.update_pose)
		self.pose = turtlesim.msg.Pose()

		# publisher for the Twist message
		self.publisher = rospy.Publisher('%s/cmd_vel' % self.turtlename, geometry_msgs.msg.Twist, queue_size=1)
		self.msg = geometry_msgs.msg.Twist()

		self.rate = rospy.Rate(15)

	def update_pose(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x, 3)
		self.pose.y = round(self.pose.y, 3)

	def follower(self, trans):
		# set angular velocity
		self.msg.angular.z = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
		# set linear velocity
		self.msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

	def go_to_goal(self, goal_x, goal_y, tolerance):
		distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
		if (distance > tolerance):
			self.msg.linear.x = 1.5*distance
			self.msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)
		else:
			self.msg.linear.x = 0
			self.msg.angular.z = 0

	def move(self):
		while not rospy.is_shutdown():

			try:
				trans = self.transformBuffer.lookup_transform(self.turtlename, 'turtle1', rospy.Time())
			# exception for error catching
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				self.rate.sleep()
				continue

			self.follower(trans)

			# publish the velocity message
			self.publisher.publish(self.msg)
			self.rate.sleep()

if __name__ == '__main__':
	try: 
		turtle = followerTurtle()
		turtle.move()
	except rospy.ROSInterruptException: pass
	rospy.spin()