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
from math import pi

class followerTurtle:
	def __init__(self):
		rospy.init_node('turtle_listener')

		self.transformBuffer = tf2_ros.Buffer()

		# listener object that received tf2 transform messages
		listener = tf2_ros.TransformListener(self.transformBuffer)

		# spawn service creates multiple turtles
		rospy.wait_for_service('spawn')
		spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)

		# get parameters from the parameter server
		self.turtlename = rospy.get_param('turtle', 'turtle2')

		# spawn turtle at the transform specified
		spawner(1, 1, 0.5*pi, self.turtlename)

		# publisher for the Twist message
		self.publisher = rospy.Publisher('%s/cmd_vel' % self.turtlename, geometry_msgs.msg.Twist, queue_size=1)
		self.msg = geometry_msgs.msg.Twist()
		self.rate = rospy.Rate(10)

	def follow(self, trans):
		# set angular velocity
		self.msg.angular.z = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
		# set linear velocity
		self.msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
		

	def move(self):
		while not rospy.is_shutdown():

			try:
				trans = self.transformBuffer.lookup_transform(self.turtlename, 'turtle1', rospy.Time())
			# exception for error catching
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				self.rate.sleep()
				continue

			self.follow(trans)

			# publish the velocity message
			self.publisher.publish(self.msg)
			self.rate.sleep()

if __name__ == '__main__':
	try: 
		turtle = followerTurtle()
		turtle.move()
	except rospy.ROSInterruptException: pass
	rospy.spin()