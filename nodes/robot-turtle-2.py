#!/usr/bin/env python

"""
Script for listening to tf2 and following the leader turtle

Haritha Murali (hm535)
30 September 2018
"""

import rospy
import math
import tf2_ros
from geometry_msgs.msg import Twist, TransformStamped
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

class Robot:
    def __init__(self):
        rospy.init_node('robot_listener')

        # get parameters from the parameter server
        self.robot = rospy.get_param('turtle', 'turtle2')
        self.human = rospy.get_param('turtle', 'turtle1')

        # spawn service to create multiple turtles
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', Spawn)
        spawner(1, 1, 0.5*math.pi, self.robot)

        # # listener for human pose
        # self.transformBuffer = tf2_ros.Buffer()
        # self.humanTransform = TransformStamped()
        # listener = tf2_ros.TransformListener(self.transformBuffer)

        # subscriber for self pose
        self.pose = Pose()
        self.subscriber = rospy.Subscriber('%s/pose' %self.robot, Pose, self.update_pose)

        # subscriber for human pose
        self.human_pose = Pose()
        self.humanSubscriber = rospy.Subscriber('%s/pose' %self.human, Pose, self.update_human_pose)
        
        # publisher for twist message
        self.msg = Twist()
        self.publisher = rospy.Publisher('%s/cmd_vel' % self.robot, Twist, queue_size = 1)

        # social cost from human
        self.socialCost = []
        # heuristic cost for goal configuration
        self.heuristicCost = []
        # total cost
        self.totalCost = []
        # initialize all costs to 0
        tmp = []
        for ii in range(0, 100):
            tmp.append(0)
        for ii in range(0, 100):
            self.socialCost.append(tmp[:])
            self.heuristicCost.append(tmp[:])
            self.totalCost.append(tmp[:])

        
        self.goal_flag = False
        self.subgoal_flag = False
        
        self.rate = rospy.Rate(20)
        
    def update_pose(self, data):
        self.pose = data 
        self.pose.x = round(self.pose.x, 3)
        self.pose.y = round(self.pose.y, 3)

    def update_human_pose(self, data):
        self.human_pose = data
        self.human_pose.x = round(self.human_pose.x, 3)
        self.human_pose.y = round(self.human_pose.y, 3)
        self.get_social_cost()
        self.get_total_cost()

    # get human pose in self (robot) frame
	# round the x and y position to prevent "jerky" movement
    # def get_human_transform(self):
    #     self.humanTransform = self.transformBuffer.lookup_transform(self.robot, 'turtle1', rospy.Time())
    #     self.humanTransform.transform.translation.x = round(self.humanTransform.transform.translation.x, 3)
    #     self.humanTransform.transform.translation.y = round(self.humanTransform.transform.translation.y, 3)

    def get_social_cost(self):
        for ii in range(0, len(self.socialCost)):
            y = float(ii/10.0)
            for jj in range(0, len(self.socialCost[0])):
                x = float(jj/10.0)
                self.socialCost[ii][jj] = 255.0*self.calculate_gaussian(x, y)
    
    def clear_map(self, cost_map):
        for ii in range(0, len(cost_map)):
            for jj in range(0, len(cost_map[0])):
                cost_map[ii][jj] = 0

    def calculate_gaussian(self, x, y):
        mean_x = self.human_pose.x #self.humanTransform.transform.translation.x
        mean_y = self.human_pose.y #self.humanTransform.transform.translation.y 
        variance_x = 0.0625
        variance_y = 0.0625
        g = (((x-mean_x)**2)/(2*variance_x)) + (((y-mean_y)**2)/(2*variance_y)) 
        p = math.exp(-g)
        return p
    
    def get_heuristic_cost(self, goal_x, goal_y):
        for ii in range(0, len(self.heuristicCost)):
            y = float(ii/10.0)
            for jj in range(0, len(self.heuristicCost[0])):
                x = float(jj/10.0)
                distance = math.sqrt((goal_x - x)**2 + (goal_y - y)**2)
                self.heuristicCost[jj][ii] = 20.0*distance
    
    def get_total_cost(self):
        for ii in range(0, len(self.totalCost)):
			for jj in range(0, len(self.totalCost[0])):
				self.totalCost[ii][jj] = self.socialCost[ii][jj] + self.heuristicCost[ii][jj]

    def get_sub_goal(self):
        tx = min(max(int(self.pose.x * 10) - 16, 0), 68)
        ty = min(max(int(self.pose.y * 10) - 16, 0), 68)

        min_cost = 1e6
        min_x = tx
        min_y = ty
        for ii in range(0,31):
            row = ty + ii
            for jj in range(0,31):
                col = tx + jj
                # print(row, col, self.totalCost[row][col])
                if(self.totalCost[row][col] < min_cost):
                    min_cost = self.totalCost[row][col]
                    min_row = row
                    min_col = col 
        
        next_x = float(min_col/10.0)
        next_y = float(min_row/10.0)
        return(next_x, next_y, min_cost)
    
    def go_to_goal(self, goal_x, goal_y):
        distance = self.calculate_euclidean(goal_x, goal_y)
        self.msg.linear.x = distance # linear speed is proportional to distance
        self.msg.angular.z = 4*(math.atan2(goal_y- self.pose.y, goal_x - self.pose.x) - self.pose.theta)
    
    def stop_moving(self):
		self.msg.linear.x = 0
		self.msg.angular.z = 0

    def calculate_euclidean(self, goal_x, goal_y):
        distance = math.sqrt((goal_x - self.pose.x)**2 + (goal_y - self.pose.y)**2)
        return distance

if __name__ == '__main__':
    turtle = Robot()
    rospy.sleep(2.0)
    # set goals
    goal_x = 5.5
    goal_y = 5.5
    tolerance = 0.05

    # calculate corresponding heuristic cost
    turtle.get_heuristic_cost(goal_x, goal_y)

    # get first subgoal
    #get total cost
    turtle.get_total_cost()

    (sub_x, sub_y, sub_cost) = turtle.get_sub_goal()
    print(sub_x, sub_y, sub_cost)

    try:
        # while not reached goal
        while (not turtle.goal_flag):

            if ( abs(goal_x - turtle.human_pose.x) < 0.1 and abs(goal_y - turtle.human_pose.y) < 0.1):
                print('changing goal as human is too close')
                goal_x -= 0.5
                goal_y -= 0.5
                # update heuristic costs
                turtle.get_heuristic_cost(goal_x, goal_y)

            # if subgoal reached, update subgoal
            distance = turtle.calculate_euclidean(sub_x, sub_y)
            if (distance < tolerance):
                (sub_x, sub_y, sub_cost) = turtle.get_sub_goal()
                print(sub_x, sub_y, sub_cost)
            
            turtle.go_to_goal(sub_x, sub_y)
            turtle.publisher.publish(turtle.msg)

            # if goal reached quit
            distance_goal = turtle.calculate_euclidean(goal_x, goal_y)
            if(distance_goal < tolerance):
                turtle.goal_flag = True
                rospy.spin()
            
            turtle.rate.sleep()

    except rospy.ROSInterruptException: 
        pass
        rospy.spin()