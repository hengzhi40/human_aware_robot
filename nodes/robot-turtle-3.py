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
        spawner(0.5, 0.5, 0.5*math.pi, self.robot)

        # subscribers and publishers
        self.robot_pose = Pose()
        self.robot_subscriber = rospy.Subscriber('%s/pose' %self.robot, Pose, self.update_robot_pose)

        self.human_pose = Pose()
        self.human_subscriber = rospy.Subscriber('%s/pose' %self.human, Pose, self.update_human_pose)
        self.human_pose_history = [ [5, 5, 5, 5, 5], [5, 5, 5, 5, 5]]
        self.time_stamps = [ -0.4, -0.3, -0.2, -0.1, 0 ]
        self.slope = [ 0, 0 ]
        self.intercept = [5.5, 5.5]

        self.vel_msg = Twist()
        self.publisher = rospy.Publisher('%s/cmd_vel' % self.robot, Twist, queue_size = 1)
        
        # cost matrices
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

        # trajectory for left to right cleaning
        self.rightward_trajectory = [ [0.5, 1.0], [0.5,9.8], [2,9.8], [2,0.5], [5,0.5], [5,9.8], [6,9.8], [6,0.5], [8,0.5], [8,9.8], [9.8,9.8], [9.8, 0.5]]

        # variable for current goal
        self.current_counter = 0
        self.current_goal = self.rightward_trajectory[self.current_counter]

        # variable for current subgoal - initialize to first goal
        self.current_subgoal = self.rightward_trajectory[self.current_counter]

        # distance to human flag
        self.distance_flag = False

        # rospy rate
        self.start_time = rospy.get_time()
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
        self.update_human_pose_history()
        self.predict_linear_trajectory()

    # update record of human poses
    def update_human_pose_history(self):
        self.human_pose_history[0].pop(0)
        self.human_pose_history[1].pop(0)
        self.time_stamps.pop(0)
        self.human_pose_history[0].append(self.human_pose.x)
        self.human_pose_history[1].append(self.human_pose.y)
        self.time_stamps.append(rospy.get_time() - self.start_time)

    # function to predict next human pose
    def predict_linear_trajectory(self):
        xbar = sum(self.human_pose_history[0]) / len(self.human_pose_history[0])
        ybar = sum(self.human_pose_history[1]) / len(self.human_pose_history[1])
        tbar = sum(self.time_stamps) / len(self.time_stamps)

        xnum = 0
        ynum = 0
        den = 0
        for i in range(0, len(self.time_stamps)):
            t1 = self.time_stamps[i] - tbar
            tx = self.human_pose_history[0][i] - xbar
            ty = self.human_pose_history[1][i] - ybar
            xnum += t1*tx
            ynum += t1*ty
            den += t1*t1
        
        mx = xnum/den
        my = ynum/den
        self.slope = [mx, my]
        self.intercept = [xbar - mx*tbar, ybar - my*tbar]

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
                
                cp0 = 255.0*self.calculate_gaussian(x, y, self.human_pose.x, self.human_pose.y, 0.0625)
                (xp, yp) = self.predict_trajectory(0.1)
                cp1 = 0.5*255.0*self.calculate_gaussian(x, y, xp, yp, 0.09375)
                (xp, yp) = self.predict_trajectory(0.2)
                cp2 = (0.5**2)*255.0*self.calculate_gaussian(x, y, xp, yp, 0.140625)
                (xp, yp) = self.predict_trajectory(0.2)
                cp3 = (0.5**3)*255.0*self.calculate_gaussian(x, y, xp, yp, 0.210)
                (xp, yp) = self.predict_trajectory(0.2)
                cp4 = (0.5**4)*255.0*self.calculate_gaussian(x, y, xp, yp, 0.315)

                self.socialCost[ii][jj] = cp0 + cp1 + cp2 + cp3 + cp4
                
    def predict_trajectory(self, time_in_sec):
        now = rospy.get_time() - self.start_time
        future = now + time_in_sec
        x_predict = self.slope[0]*future + self.intercept[0]
        y_predict = self.slope[1]*future + self.intercept[1]
        return(x_predict, y_predict)

    # gaussian function
    def calculate_gaussian(self, x, y, mean_x, mean_y, variance):
        g = (((x-mean_x)**2)/(2*variance)) + (((y-mean_y)**2)/(2*variance)) 
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

    # function to sum all costs
    def get_total_cost(self):
        for ii in range(0, len(self.totalCost)):
            for jj in range(0, len(self.totalCost[0])):
                self.totalCost[ii][jj] = self.socialCost[ii][jj] + self.heuristicCost[ii][jj] 

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
        self.current_goal = self.rightward_trajectory[self.current_counter]
        
        self.current_counter += 1
        if self.current_counter > 11:
            self.current_counter = 0    

    # function to calculate distance to current goal
    def calculate_euclidean(self, goal_x, goal_y):
        distance = math.sqrt((goal_x - self.robot_pose.x)**2 + (goal_y - self.robot_pose.y)**2)
        return distance       

    def calculate_to_human(self):
        distance = math.sqrt((self.human_pose.x - self.robot_pose.x)**2 + (self.human_pose.y - self.robot_pose.y)**2)
        return distance

    # pid controller to move turtle to (sub)goal
    def go_to_goal(self):
        # print('going to goal')
        distance =  math.sqrt((self.current_subgoal[0] - self.robot_pose.x)**2 + (self.current_subgoal[1] - self.robot_pose.y)**2)
        self.vel_msg.linear.x = min(1.0, distance) # linear speed is proportional to distance
        self.vel_msg.angular.z = 4*(math.atan2(self.current_subgoal[1]- self.robot_pose.y, self.current_subgoal[0] - self.robot_pose.x) - self.robot_pose.theta)

    # set velocity to zero if close to (sub)goal
    def stop_moving(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0

if __name__ == '__main__':
    # create instance of Robot Turtle
    turtle = Robot()
    rospy.sleep(2.0)
    tolerance = 0.05
    thresh_human = 2.5
    thresh_cost = 10000

    turtle.get_heuristic_cost()
    turtle.get_social_cost()
    turtle.get_total_cost()

    try:
        counter = 1
        while not rospy.is_shutdown():
            
            # else: turtle.distance_flag = False

            distance_goal = turtle.calculate_euclidean(turtle.current_goal[0], turtle.current_goal[1])
            if(distance_goal > tolerance):
                if(turtle.distance_flag):

                    if(counter%10 == 0):
                        turtle.stop_moving()
                        turtle.publisher.publish(turtle.vel_msg)
                        turtle.get_social_cost()
                        turtle.get_total_cost()
                        distance_human = turtle.calculate_to_human()
                        if(distance_human > thresh_human):
                            turtle.distance_flag = False
                        turtle.rate.sleep()

                    else:
                        (sub_x, sub_y, sub_cost) = turtle.get_sub_goal()
                        turtle.current_subgoal[0] = sub_x
                        turtle.current_subgoal[1] = sub_y
                        print(sub_x, sub_y, sub_cost)
                        if (sub_cost >= thresh_cost):
                            turtle.stop_moving()
                        else: turtle.go_to_goal()
                        turtle.publisher.publish(turtle.vel_msg) 
                        turtle.rate.sleep()
                else:
                    turtle.current_subgoal = turtle.current_goal

                    turtle.go_to_goal()
                    turtle.publisher.publish(turtle.vel_msg) 
                    distance_human = turtle.calculate_to_human()
                    if(distance_human <= thresh_human):
                        turtle.distance_flag = True
                    turtle.rate.sleep()

            elif(distance_goal <= tolerance):
                turtle.get_next_goal()
                turtle.get_heuristic_cost()
                turtle.rate.sleep()
            
            counter += 1
            if counter >= 10:
                counter = 0
        
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass