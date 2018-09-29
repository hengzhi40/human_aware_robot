#!/usr/bin/env python

"""
Script for broadcasting the turtle state to tf2

Haritha Murali (hm535)
21 September 2018
"""

import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg 
import turtlesim.msg 
from math import pi

class leaderTurtle:
	def __init__ (self):
		rospy.init_node('turtle_broadcaster', anonymous = True)

		# create the transform broadcaster object that will be used to send transformations
		self.broadcaster = tf2_ros.TransformBroadcaster()
		self.vel_publisher = rospy.Publisher('turtle1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

		# create the message object where the transformation will be stored
		self.t = geometry_msgs.msg.TransformStamped()

		# create pose subscriber
		self.turtlename = rospy.get_param('~turtle') #each turtle has a unique name
		subscriber = rospy.Subscriber('/%s/pose' % self.turtlename, turtlesim.msg.Pose, self.update_pose)
		
		self.pose = turtlesim.msg.Pose()
		self.vel_msg = geometry_msgs.msg.Twist()
		self.vel_msg.linear.x = 2
		self.rate = rospy.Rate(10)

	def update_pose(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x, 2)
		self.pose.y = round(self.pose.y, 2)

	def broadcast(self):
		
		while not rospy.is_shutdown():

			if (self.pose.x > 10 or self.pose.x < 0.5):
				self.vel_msg.linear.x *= -1
			
			self.t.header.stamp = rospy.Time.now()
			self.t.header.frame_id = "world"
			self.t.child_frame_id = self.turtlename
			self.t.transform.translation.x = self.pose.x
			self.t.transform.translation.y = self.pose.y
			self.t.transform.translation.z = 0.0

			q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.pose.theta)
			self.t.transform.rotation.x = q[0]
			self.t.transform.rotation.y = q[1]
			self.t.transform.rotation.z = q[2]
			self.t.transform.rotation.w = q[3]

			self.broadcaster.sendTransform(self.t)
			self.vel_publisher.publish(self.vel_msg)
			# print(self.vel_msg)
			self.rate.sleep()

if __name__ == '__main__':
	try: 
		turtle = leaderTurtle()
		turtle.broadcast()
		# turtle.broadcast_transform()
	except rospy.ROSInterruptException: pass
	rospy.spin()