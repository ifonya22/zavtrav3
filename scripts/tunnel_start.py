#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from time import sleep
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image, CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
import PIL

ranges = list()
tunnel = False
in_tunnel = False



def mask_yellow(img):
    pub_image = rospy.Publisher('image', Image, queue_size=1)
    cvBridge = CvBridge()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    Hue_l = 27
    Hue_h = 41
    Saturation_l = 130
    Saturation_h = 255
    Lightness_l = 160
    Lightness_h = 255
    # define range of yellow color in HSV
    lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
    upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask = mask)
    fraction_num = np.count_nonzero(mask)
    point_arr = []
    stop_flag = False
    if fraction_num > 50:
        k = 0
        jold = 0
        for i in range(mask.shape[0]-1,0,-15):
            if stop_flag == True:
                break
            for j in range(0,mask.shape[1],15):
                if mask[i,j] > 0:
                    point_arr.append([j,i])
                    k+=1
                    if abs(j-jold) > 80 and k > 1:
                        point_arr.pop()
                        stop_flag = True
                    jold = j
                    break
        if(len(point_arr) > 0):
            point_before = point_arr[0]
            for point in point_arr:
                res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
                point_before = point
    return res, point_arr
def calculate_error( yellow_array):
    
    error_yell = 0
    
    weight = 0
    i = 1
    for yel in yellow_array:
        #when yel[2] = 600 then weight = 0 and if yel[2] = 0 wheight = 1
        weight = yel[1]*0.0017 + 1
        error_yell = weight*(30 - yel[0]) + error_yell
        i+=1
        error_yell = error_yell/i

    return error_yell

def cbImageProjection( data):
    #translate 2 numpy
    cvBridge = CvBridge()
    pub_image = rospy.Publisher('image', Image, queue_size=1)
    pub_error = rospy.Publisher('line_error', Float64, queue_size=1)
    error = Float64()

    cv_image_original = cvBridge.imgmsg_to_cv2(data, "bgr8")
    #img will be more stable
    cv_image_original = image_transform(cv_image_original)
    cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)
    #detection line
    yellow_detect, yellow_array = mask_yellow(cv_image_original)
    detected = cv2.add( yellow_detect)
    #publish in topic image - 11 
    pub_image.publish(cvBridge.cv2_to_imgmsg(detected, "bgr8"))
    #publish in topic image - 12
    #self.error = Float64()
    error.data = calculate_error(yellow_array)
    pub_error.publish(error)
    #print(error)
    return yellow_array

def image_transform(img):
    x, y = 320, 240
    #pts1 = np.float32([[0, 0],[x, 0], [20, y/2],[x-20,y/2]]) 
    #pts1 = np.float32([[0+35, 0+125],[x-45, 0+125], [0-40, y+50],[x+60,y+50]]) 
    #pts2 = np.float32([[0,0],[x,0],[0,y],[x,y]])
    #pts1 = np.float32([[160 - 75, 180 - 35], [160 + 75, 180 - 35], [160 + 165, 120 + 120], [160 - 165, 120 + 120]]) 
    #pts2 = np.float32([[150, 0], [650, 0], [1000, 600], [200, 800]])
    #M = cv2.getPerspectiveTransform(pts1,pts2)
    #M, status = cv2.findHomography(pts1,pts2)
    #triangle1 = np.array([[0, 599], [0+250, 340], [200, 599]], np.int32)
    #triangle2 = np.array([[999+300, 599], [999+650, 340], [799+200, 599]], np.int32)
    #black = (0, 0, 0)
    #cv_image_homography = cv2.warpPerspective(img,M,(1000, 600))
    #white = (255, 255, 255)
    #cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)
    #return cv_image_homography
    #return cv2.warpPerspective(img,M,(1000, 600))

    ## homography transform process
    # selecting 4 points from the original image
    #pts_src = np.float32([[160 - 75, 180 - 35], [160 + 75, 180 - 35], [160 + 165, 120 + 120], [160 - 165, 120 + 120]])
    #pts_src = np.float32([[160 , 120], [0, 320 - 75], [320, 240], [0, 240]])
    pts_src = np.float32([[160 -75 , 120 -35], [160 +75, 120 -35], [160+160, 120 +120], [160 -160, 120 +120]])
    #pts_dst = np.float32([[75, 0], [325, 0], [500, 300], [100, 400]])
    # selecting 4 points from image that will be transformed
    pts_dst = np.float32([[100, 0], [320, 0], [320, 100], [50, 200]])
    h = cv2.getPerspectiveTransform(pts_src,pts_dst)


    # finding homography matrix
    #h, status = cv2.findHomography(pts_src, pts_dst)

    # homography process
    cv_image_homography = cv2.warpPerspective(img, h, (320, 240))

    #triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
    #triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
    triangle1 = np.array([[0, 240], [0, 320], [200, 240]], np.int32)
    triangle2 = np.array([[320, 240], [320, 320], [200, 240]], np.int32)
    black = (0, 0, 0)
    white = (255, 255, 255)
    cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)

    im = PIL.Image.fromarray(img)    
    area = (0,160, 320,240) 
    im = im.crop(area) 
    #im = im.resize((320,240))
    im = np.array(im)
    #im = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
    return im

def cb_sign(data):
    global tunnel
    if(data.data == "tunnel"):
        tunnel = True

def cb_scan(data):
    global ranges
    ranges = data.ranges

def pub_velocity(x, z, time):
    pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    vel = Twist()
    for i in range(0, int(time*10)):
        vel.linear.x = x
        vel.angular.z = z
        pub_vel.publish(vel)
        rospy.sleep(0.1)

def do_tunnel():
    pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
    flag_move_line = Bool()
    flag_move_line.data = False
    rospy.sleep(4) #3
    rospy.sleep(0.1)
    pub_line_move.publish(flag_move_line)
    pub_velocity(0.2,0,2)
    for i in range(20,0, -2):
        pub_velocity(i/100,0,0.3)
    

def sign(data):
    if(data >= 0):
        return 1
    else:
        return -1

def error_handler(msg):
    if msg.data > 0:
        pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
        a = Bool()
        a=True
        pub_line_move.publish(a)

    

def in_tunnel_go():
    global ranges
    vel_x = 0.12
    vel_z = 0
    error = 4*(0.23 - ranges[300])
    if(abs(error) > 1.5):
        error = sign(error)*1.5
    vel_z = error
    for i in range(0,360,1):
        if(ranges[i] < 0.15):
            if(i <= 30 or i >= 330):
                vel_x = -0.09
                vel_z = 0.4        
    pub_velocity(vel_x, vel_z, 0.1)
  #  sub_image = rospy.Subscriber('line_error', Float64, error_handler, queue_size=1)
    
    pub_image = rospy.Subscriber('image', Image, queue_size=1)
    f = Image()
##  
    point = mask_yellow(f)
    yellow_array= cbImageProjection(point)
    errorr = calculate_error(yellow_array)
    if errorr > 0 :
        pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
        a = Bool()
        a=True
        pub_line_move.publish(a)


if __name__ == '__main__':
    rospy.init_node('tunnel')
    sub_sign = rospy.Subscriber('sign', String, cb_sign, queue_size=1)
    sub_bar = rospy.Subscriber('scan', LaserScan, cb_scan, queue_size=1)
    while not rospy.is_shutdown():
        try:
            if(tunnel == True and in_tunnel == False):
                print("tunnel detected")
                do_tunnel()
                in_tunnel = True
            elif(in_tunnel == True):
                in_tunnel_go()
        except KeyboardInterrupt:
            break