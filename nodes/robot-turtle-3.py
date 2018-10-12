#!/usr/bin/env python

"""
Program for socially navigating robots

Haritha Murali (hm535)
11 October 2018
"""

import rospy
import math
from geometry_msgs.msg import Twist, TransformStamped
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

class Robot:
    def __init__(self):
        rospy.init_node('robot_turtle')

        # get parameters from server
        self.robot = rospy.get_param('turtle', 'turtle2')
        self.human = rospy.get_param('turtle', 'turtle1')

        # spawn to get multiple turtles
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', Spawn)
        spawner(0, 0, 0.5*math.pi, self.robot)

        # subscribers and publishers
        self.robot_pose = Pose()
        self.robot_subscriber = rospy.Subscriber('%s/pose' %self.robot, Pose, self.update_robot_pose)

        self.human_pose = Pose()
        self.human_subscriber = rospy.Subscriber('%s/pose' %self.human, Pose, self.update_human_pose)

        self.vel_msg = Twist()
        self.publisher = rospy.Publisher('%s/cmd_vel' % self.robot, Twist, queue_size = 1)
        
        # cost matrices
        # social cost from human
        self.socialCost = []
        # heuristic cost for goal configuration
        self.heuristicCost = []
        # distance cost from self
        self.distanceCost = []
        # total cost
        self.totalCost = []

        # initialize all costs to 0
        tmp = []
        for ii in range(0, 100):
            tmp.append(0)
        for ii in range(0, 100):
            self.socialCost.append(tmp[:])
            self.heuristicCost.append(tmp[:])
            self.distanceCost.append(tmp[:])
            self.totalCost.append(tmp[:])

        # direction of window cleaning -> left to right or right to left?
        self.direction = True

        # trajectory for left to right cleaning
        self.rightward_trajectory = [ [0.5,0.5], [0.5,9.8], [2,9.8], [2,0.5], [5.2,0.5], [5.4,9.8], [6,9.8], [6,0.5], [8,0.5], [8,9.8], [9.8,9.8]]

        # trajectory for right to left cleaning
        self.leftward_trajectory = [[10,10], [10,0], [8,0], [8,10], [6,10], [6,0], [4,0], [4,10], [2,10], [2,0], [0,0]]

        # variable for current goal
        self.current_counter = 0
        self.current_goal = self.rightward_trajectory[self.current_counter]

        # variable for current subgoal
        self.current_subgoal = self.rightward_trajectory[self.current_counter]

        # rospy rate
        self.rate = rospy.Rate(10)

    # callback function to update self pose
    def update_robot_pose(self, data):
        self.robot_pose = data
        self.robot_pose.x = round(self.robot_pose.x, 3)
        self.robot_pose.y = round(self.robot_pose.y, 3)

    # callback function to update human pose
    def update_human_pose(self, data):
        self.human_pose = data
        self.human_pose.x = round(self.human_pose.x, 3)
        self.human_pose.y = round(self.human_pose.y, 3)

    # function to clear cost matrices
    def clear_map(self, cost_map):
        for ii in range(0, len(cost_map)):
            for jj in range(0, len(cost_map[0])):
                cost_map[ii][jj] = 0

    # function to calculate social costs
    def get_social_cost(self):
        for ii in range(0, len(self.socialCost)):
            y = float(ii/10.0)
            for jj in range(0, len(self.socialCost[0])):
                x = float(jj/10.0)
                self.socialCost[ii][jj] = 255.0*self.calculate_gaussian(x, y)

    # gaussian function
    def calculate_gaussian(self, x, y):
        mean_x = self.human_pose.x #self.humanTransform.transform.translation.x
        mean_y = self.human_pose.y #self.humanTransform.transform.translation.y 
        variance_x = 0.0625
        variance_y = 0.0625
        g = (((x-mean_x)**2)/(2*variance_x)) + (((y-mean_y)**2)/(2*variance_y)) 
        p = math.exp(-g)
        return p

    # function to calculate heuristic cost to current goal
    def get_heuristic_cost(self):
        for ii in range(0, len(self.heuristicCost)):
            y = float(ii/10.0)
            for jj in range(0, len(self.heuristicCost[0])):
                x = float(jj/10.0)
                distance = math.sqrt((self.current_goal[0] - x)**2 + (self.current_goal[1] - y)**2)
                self.heuristicCost[ii][jj] = 20.0*distance

    # function to calculate euclidean distance cost
    def get_distance_cost(self):
        for ii in range(0, len(self.distanceCost)):
            y = float(ii/10.0)
            for jj in range(0, len(self.distanceCost[0])):
                x = float(jj/10.0)
                distance = math.sqrt((self.robot_pose.x - x)**2 +  (self.robot_pose.y - y)**2)
                self.distanceCost[ii][jj] = 1600.0*distance

    # function to sum all costs
    def get_total_cost(self):
        for ii in range(0, len(self.totalCost)):
            for jj in range(0, len(self.totalCost[0])):
                self.totalCost[ii][jj] = self.socialCost[ii][jj] + self.heuristicCost[ii][jj] + self.distanceCost[ii][jj]

    # function to search for next best subgoal
    def get_sub_goal(self):
        tx = min(max(int(self.robot_pose.x * 10) - 16, 0), 68)
        ty = min(max(int(self.robot_pose.y * 10) - 16, 0), 68)

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

    # function to update current goal
    def get_next_goal(self):
        if(self.direction):
            self.current_goal = self.rightward_trajectory[self.current_counter]
        elif(not self.direction):
            self.current_goal = self.leftward_trajectory[self.current_counter]
        
        self.current_counter += 1
        if self.current_counter > 10:
            self.current_counter = 10
            # self.direction = not self.direction      

    # function to calculate distance to current goal
    def calculate_euclidean(self, goal_x, goal_y):
        distance = math.sqrt((goal_x - self.robot_pose.x)**2 + (goal_y - self.robot_pose.y)**2)
        return distance       

    # pid controller to move turtle to (sub)goal
    def go_to_goal(self):
        # print('going to goal')
        distance =  math.sqrt((self.current_subgoal[0] - self.robot_pose.x)**2 + (self.current_subgoal[1] - self.robot_pose.y)**2)
        self.vel_msg.linear.x = distance # linear speed is proportional to distance
        self.vel_msg.angular.z = 4*(math.atan2(self.current_subgoal[1]- self.robot_pose.y, self.current_subgoal[0] - self.robot_pose.x) - self.robot_pose.theta)

    # set velocity to zero if close to (sub)goal
    def stop_moving(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0

if __name__ == '__main__':
    # create instance of Robot Turtle
    turtle = Robot()
    rospy.sleep(2.0)
    tolerance = 0.025

    turtle.get_heuristic_cost()
    turtle.get_social_cost()
    turtle.get_total_cost()

    (sub_x, sub_y, sub_cost) = turtle.get_sub_goal()
    print(turtle.current_subgoal)

    try:
        while not rospy.is_shutdown():
            distance_goal = turtle.calculate_euclidean(turtle.current_goal[0], turtle.current_goal[1])
            if(distance_goal > tolerance):
                distance_subgoal = turtle.calculate_euclidean(turtle.current_subgoal[0], turtle.current_subgoal[1])
                # print(turtle.current_goal)
                if (distance_subgoal < tolerance):
                    (sub_x, sub_y, sub_cost) = turtle.get_sub_goal()
                    turtle.current_subgoal[0] = sub_x
                    turtle.current_subgoal[1] = sub_y
                    print(turtle.current_subgoal)
                
                turtle.get_social_cost()
                turtle.get_total_cost()
                turtle.go_to_goal()
                turtle.publisher.publish(turtle.vel_msg)
                turtle.rate.sleep()
            
            elif(distance_goal <= tolerance):
                turtle.get_next_goal()
                turtle.get_heuristic_cost()
                turtle.rate.sleep()
            
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    