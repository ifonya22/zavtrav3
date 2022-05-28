#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import Float64, Bool
import numpy as np
from operator import itemgetter
from glob import glob
import matplotlib.pyplot as plt
import PIL
from geometry_msgs.msg import Twist
class LineChecker:
    
    def __init__(self):
        rospy.init_node('image_projection')
        
        #self.sub_image = rospy.Subscriber('/camera_line/image_line', Image, self.cbImageProjection, queue_size=1)
        self.pub_image = rospy.Publisher('image', Image, queue_size=1)
        self.pub_error = rospy.Publisher('line_error', Float64, queue_size=1)
        self.cvBridge = CvBridge()
        self.error = Float64()
        
        print('go')
        
    def error_callback(self, msg):
        #print(msg.data)
        print('check')
    
    def cbImageProjection(self, data):
        #translate 2 numpy
        
        cv_image_original = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
        #img will be more stable
        cv_image_original = self.image_transform(cv_image_original)
        cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)
        #detection line
        yellow_detect, yellow_array = self.mask_yellow(cv_image_original)
        white_detect, white_array = self.mask_white(cv_image_original)
        detected = cv2.add(white_detect, yellow_detect)
        #publish in topic image - 11 
        self.pub_image.publish(self.cvBridge.cv2_to_imgmsg(detected, "bgr8"))
        #publish in topic image - 12
        #self.error = Float64()
        self.error.data = self.calculate_error(yellow_array, white_array)
        self.pub_error.publish(self.error)
        print(self.error)

    def mask_yellow(self, img):
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

    def mask_white(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        Hue_l = 0
        Hue_h = 25
        Saturation_l = 0
        Saturation_h = 36
        Lightness_l = 180
        Lightness_h = 255
        # define range of yellow color in HSV
        lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_white = np.array([Hue_h, Saturation_h, Lightness_h])
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_white, upper_white)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask = mask)
        fraction_num = np.count_nonzero(mask)
        point_arr = []
        stop_flag = False
        if fraction_num > 50:
            k = 0
            jold = 0
            # last param we can change
            for i in range(mask.shape[0]-1,0,-20):
                if stop_flag == True:
                    break
                for j in range(mask.shape[1]-1,0,-20):
                    if mask[i,j] > 0:
                        point_arr.append([j,i])
                        k+=1
                        # we can change this param
                        if abs(j-jold) > 80 and k > 1:
                            point_arr.pop()
                            stop_flag = True
                        jold = j
                        break
            if len(point_arr) > 0:
                point_before = point_arr[0]
                for point in point_arr:
                    res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
                    point_before = point
        return res, point_arr


    def calculate_error(self, yellow_array, white_array):
        
        error_yell = 0
        error_white = 0
        weight = 0
        i = 1
        for yel in yellow_array:
            #when yel[2] = 600 then weight = 0 and if yel[2] = 0 wheight = 1
            weight = yel[1]*0.0017 + 1
            error_yell = weight*(30 - yel[0]) + error_yell
            i+=1
            error_yell = error_yell/i
        for white in white_array:
            weight = white[1]*0.0017 + 1
            error_white = weight*(300 - white[0]) + error_white
            i+=1
        error_white = error_white/i
        print("white "+ str(error_white) + " yellow "+ str(error_yell))
        if error_white < 30:
            return error_yell
        elif error_yell < 30:
            return error_white
        else:
            return (error_white + error_yell)/2
        

    def image_transform(self, img):
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

    def nespin(self):
        #sub_image = rospy.Subscriber('/camera_line/image_line', Image, self.cbImageProjection, queue_size=1)
        sub_image = rospy.Subscriber('/camera/image', Image, self.cbImageProjection, queue_size=1)
        
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                break
                cv2.destroyAllWindows()


if __name__ == '__main__':
    lc = LineChecker()
    lc.nespin()
    # adin = rospy.Subscriber('/line_dont_move_flag', Bool, queue_size=1)
    # pub_line_dont_move = rospy.Publisher('line_dont_move_flag', Bool, queue_size=1)
    # flag_dont_move_line = Bool()
    # flag_dont_move_line.data = False
    # pub_line_dont_move.publish(flag_dont_move_line)
    # if adin:
    #     lc.nespin()
    # else:
    #     cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    #     twist = Twist()
    #     twist.linear.x = 0.21
    #     twist.angular.z= -0.15
    #     cmd_pub.publish(twist)
