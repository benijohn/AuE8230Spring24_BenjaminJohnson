#!/usr/bin/env python3
import rospy 
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import math

class turtlebot():
    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.vel = Twist()
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def moveInCircle(self):
            speed = 0.1
            radius = 1
            omega = speed/radius
            distance = 2*math.pi*radius

            #Checking if the movement is forward or backwards
            self.vel.linear.x= speed
            self.vel.angular.z = omega
            print(omega)

            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0

            #Loop to move the turtle in an specified distance
            while(current_distance < distance):
                #Publish the velocity
                self.update_pose(self)
                self.velocity_publisher.publish(self.vel)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= speed*(t1-t0)
            #After the loop, stops the robot
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            #Force the robot to stop
            self.velocity_publisher.publish(self.vel)
   
            


if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.moveInCircle()

    except rospy.ROSInterruptException: pass