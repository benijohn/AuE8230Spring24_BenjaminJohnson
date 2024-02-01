#!/usr/bin/env python3
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,cos,sin
import numpy as np
import math

class turtlebot():
    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = float(input("Set your x goal:"))
        goal_pose.y = float(input("Set your y goal:"))
        distance_tolerance = float(input("Set your tolerance:"))
        vel_msg = Twist()

        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z =0
        self.velocity_publisher.publish(vel_msg)

        rospy.spin()

    def moveInSquare(self):
        # pose = self.pose
        vel_msg = Twist()
        # length = float(input("Please input edge length:"))
        # edge_tol = float(input("Please input a tolerence for edge length:"))
        # direction = float(input("Clockwise? (1 for yes, 0 for no)"))
        # theta_tol = float(input("Please input a tolerance for corner angle:"))

        # Get the input from the user.
        #goal_pose.x = float(input("Set your x goal: "))
        length = rospy.get_param('~edge_length')
        #goal_pose.y = float(input("Set your y goal: "))
        edge_tol = rospy.get_param('~edge_tol')
        direction = rospy.get_param('~direction')
        theta_tol = rospy.get_param('~theta_tol')

        print(length)
        print(edge_tol)
        print(direction)
        

        coords = np.ndarray([4,3])
        # print(pose)
        x0 = self.pose.x
        y0 = self.pose.y
        theta0 = self.pose.theta

        if(direction):
            for i in range(4):
                x0 = x0 + length*(cos(theta0))
                y0 = y0 + length*(sin(theta0))
                theta0 = theta0 + math.pi/2
                coords[i,:] = [x0,y0,theta0]
        else:
            for i in range(4):
                x0 = x0 + length*(cos(theta0))
                y0 = y0 + length*(sin(theta0))
                theta0 = theta0 - math.pi/2
                coords[i,:] = [x0,y0,theta0]       

        print(coords)

        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        for i in range(4):
            print(i)
            x_goal = coords[i,0]
            y_goal = coords[i,1]
            theta_goal = coords[i,2]

            print(coords)
            print(self.pose)
            while sqrt(pow((x_goal - self.pose.x), 2) + pow((y_goal - self.pose.y), 2)) >= edge_tol:
                print(coords)
                print(self.pose)
                print(atan2(y_goal - self.pose.y, x_goal - self.pose.x))
                vel_msg.linear.x = 1.5 * sqrt(pow((x_goal - self.pose.x), 2) + pow((y_goal - self.pose.y), 2))
                print(vel_msg)
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)
            while(abs(theta_goal-self.pose.theta) > theta_tol):
                print(abs(theta_goal-self.pose.theta))
                vel_msg.angular.z = 2 * (theta_goal - self.pose.theta)
                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)    

            # x2 = x1 + length*(cos(theta1))
            # y2 = y1 + length*(sin(theta1))
            # theta2 = theta1+math.pi/2
            


if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        #x.move2goal()
        x.moveInSquare()

    except rospy.ROSInterruptException: pass