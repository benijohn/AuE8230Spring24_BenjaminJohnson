#!/usr/bin/env python3
# from opertator import call
import rospy
import math
# from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

     
def move():
    # Starts a new node
    rospy.init_node('turtlebot_controller', anonymous=False)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate = rospy.Rate(100)

    #Receiveing the user's input
    speed = 0.3
    rotation_speed = 0.3
    distance = 2

    #Checking if the movement is forward or backwards
    for i in range(4):
        vel_msg.linear.x = speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        # while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            rate.sleep()
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)
        # rotate the robot
        vel_msg.angular.z = abs(rotation_speed)
        t0 = rospy.Time.now().to_sec()
        current_rotation = 0
        while(abs(current_rotation) < math.pi/2):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_rotation = rotation_speed*(t1-t0)
        vel_msg.angular.z = 0    
        velocity_publisher.publish(vel_msg)    


if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass