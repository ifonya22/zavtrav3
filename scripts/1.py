import numpy as np
import cv2
from operator import itemgetter
from glob import glob
import matplotlib.pyplot as plt
paper = cv2.imread(r"/home/ilya/catkin_ws/1.png")
# Coordinates that you want to Perspective Transform
#pts1 = np.float32([[0, 366],[500,366],[0, 0],[500, 0]])
pts1 = np.float32([[0+80, 0],[300, 0+35], [0, 206-80],[300,206]]) #500x366
# Size of the Transformed Image
pts2 = np.float32([[0,0],[320,0],[0,240],[320,240]])

M = cv2.getPerspectiveTransform(pts1,pts2)
dst = cv2.warpPerspective(paper,M,(320,240))
cv2.imshow('', dst)
cv2.waitKey(0)
cv2.destroyAllWindows()

def image_transform(self, img):
    pts1 = np.float32([[0, 0],[320, 0], [0, 200],[280,240]]) 
    pts2 = np.float32([[0,0],[320,0],[0,240],[320,240]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    return cv2.warpPerspective(paper,M,(320,240))