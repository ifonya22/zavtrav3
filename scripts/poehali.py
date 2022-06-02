#!/usr/bin/env python
# -*- coding: utf-8 -*-
from distutils.file_util import move_file
from cv2 import integral
from matplotlib.pyplot import twinx
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
import cv2

from nav_msgs.msg import Odometry


class Solver:

    def __init__(self):
        rospy.init_node('solver')
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.on_shutdown(self.on_shut)

        self.x_vel = 0

    def odom_callback(self, msg):
        print(msg)
        self.x_vel = msg.twist.twist.linear.x 

    def spin(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            
            twist = Twist()
            twist.linear.x = 0.1

            self.cmd_pub.publish(twist)

            rate.sleep()

    def on_shut(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

class LineController:

    def __init__(self):
        #rospy.init_node('line_control')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.integral = 0
        self.proportional = 0
        self.differential = 0
        self.move_flag = True
        rospy.Subscriber('/line_error', Float64,  self.error_callback)


    def error_callback(self, msg):
        print(msg)



    def errorMininimazer(self, error):
<<<<<<< Updated upstream
        Kd = 0.0039
=======
        Kd = 0.0045#0.0039
>>>>>>> Stashed changes
        Kp = 0.00321
        Ki = 0.00001
        if(self.move_flag == True):
            twist = Twist()
            self.integral = self.integral + Ki*error.data #0.000005*error.data
            self.differential = Kd * error.data
            self.proportional = Kp * error.data #0.025
            up = self.proportional +  self.differential + self.integral
            twist.angular.z = up
            #twist.linear.x = (0.22 - 0.09*abs(up)) #0.09
            twist.linear.x = (0.22 - 0.09*abs(up))
            self.cmd_pub.publish(twist)
    
    def errorFlag(self, data):
        
        self.move_flag = data.data

    def lineControl(self):
        sub_image = rospy.Subscriber('line_error', Float64, self.errorMininimazer, queue_size=1)
        sub_move_flag = rospy.Subscriber('line_move_flag', Bool, self.errorFlag, queue_size=1)
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                break
    
if __name__=="__main__":
    s = Solver()
    lc = LineController()
    #s.spin()
    lc.lineControl()

